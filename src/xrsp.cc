#include "xrsp.h"

#include <fmt/core.h>
#include <libusb.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <optional>
#include <cstdlib>
#include <thread>

namespace xrsp {

namespace {
constexpr uint16_t kVendorId = 0x2833;  // Meta/Oculus
constexpr uint16_t kProductId = 0x0186; // Meta Quest device
constexpr int kReadChunk = 0x200;
constexpr int kUsbTimeoutMs = 200;

// verbose logging controlled by XRSP_LOG env var (non-zero enables)
bool IsVerboseLoggingEnabled() {
  static int cached = -1;
  if (cached == -1) {
    const char* v = std::getenv("XRSP_LOG");
    cached = (v && v[0] != '\0' && !(v[0] == '0' && v[1] == '\0')) ? 1 : 0;
  }
  return cached == 1;
}

template <typename... Args>
inline void VPrint(fmt::format_string<Args...> fmtstr, Args&&... args) {
  if (IsVerboseLoggingEnabled()) {
    fmt::print(fmtstr, std::forward<Args>(args)...);
  }
}

uint64_t NowNs() {
  using namespace std::chrono;
  return duration_cast<nanoseconds>(steady_clock::now().time_since_epoch())
      .count();
}

uint16_t LoadLE16(const uint8_t* p) {
  return static_cast<uint16_t>(p[0] | (p[1] << 8));
}
uint32_t LoadLE32(const uint8_t* p) {
  return static_cast<uint32_t>(p[0] | (p[1] << 8) | (p[2] << 16) |
                               (p[3] << 24));
}
uint64_t LoadLE64(const uint8_t* p) {
  return static_cast<uint64_t>(LoadLE32(p)) |
         (static_cast<uint64_t>(LoadLE32(p + 4)) << 32);
}

void StoreLE16(uint16_t v, std::vector<uint8_t>& out) {
  out.push_back(static_cast<uint8_t>(v & 0xFF));
  out.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
}
void StoreLE32(uint32_t v, std::vector<uint8_t>& out) {
  out.push_back(static_cast<uint8_t>(v & 0xFF));
  out.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
  out.push_back(static_cast<uint8_t>((v >> 16) & 0xFF));
  out.push_back(static_cast<uint8_t>((v >> 24) & 0xFF));
}
void StoreLE64(uint64_t v, std::vector<uint8_t>& out) {
  StoreLE32(static_cast<uint32_t>(v & 0xFFFFFFFFull), out);
  StoreLE32(static_cast<uint32_t>((v >> 32) & 0xFFFFFFFFull), out);
}

bool IsBulk(const libusb_endpoint_descriptor& ep) {
  return (ep.bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) ==
         LIBUSB_TRANSFER_TYPE_BULK;
}

}  // namespace

// TopicPacket ---------------------------------------------------------------

TopicPacket::TopicPacket(const std::vector<uint8_t>& buffer) {
  ParseHeader(buffer);
}

void TopicPacket::ParseHeader(const std::vector<uint8_t>& buffer) {
  if (buffer.size() < 8) {
    header_ = {};
    payload_.clear();
    remainder_bytes_.clear();
    real_size_ = 0;
    full_size_ = 0;
    return;
  }

  header_.topic_raw = LoadLE16(&buffer[0]);
  header_.num_words = LoadLE16(&buffer[2]);
  header_.sequence = LoadLE16(&buffer[4]);

  header_.version_low3 = static_cast<uint8_t>(header_.topic_raw & 0x7);
  header_.has_align_padding = (header_.topic_raw & 0x8) != 0;
  header_.packet_version_is_internal = (header_.topic_raw & 0x10) != 0;
  header_.packet_version_number = (header_.topic_raw & 0x20) != 0;
  header_.topic_idx = static_cast<uint8_t>((header_.topic_raw >> 8) & 0x3F);
  header_.unk_14_15 = static_cast<uint8_t>((header_.topic_raw >> 14) & 0x3);

  full_size_ = (header_.num_words > 0 ? (header_.num_words - 1) * 4ull : 0ull);
  real_size_ = full_size_;

  FinalizePayloadViews(buffer);
}

void TopicPacket::FinalizePayloadViews(const std::vector<uint8_t>& buffer) {
  const size_t payload_offset = 8;
  const size_t total_needed = payload_offset + full_size_;

  if (buffer.size() < total_needed) {
    // Incomplete payload, keep what we have.
    const size_t avail =
        buffer.size() > payload_offset ? buffer.size() - payload_offset : 0;
    payload_.assign(
        buffer.begin() + static_cast<ptrdiff_t>(payload_offset),
        buffer.begin() + static_cast<ptrdiff_t>(payload_offset + avail));
    remainder_bytes_.clear();
    return;
  }

  payload_.assign(
      buffer.begin() + static_cast<ptrdiff_t>(payload_offset),
      buffer.begin() + static_cast<ptrdiff_t>(payload_offset + full_size_));

  if (header_.has_align_padding && !payload_.empty()) {
    const uint8_t pad = payload_.back();
    // real_size -= payload[last], then truncate
    if (pad <= payload_.size()) {
      real_size_ -= pad;
      payload_.resize(real_size_);
    }
  }

  // everything beyond the full size is remainder (including trailing frames)
  remainder_bytes_.assign(buffer.begin() + static_cast<ptrdiff_t>(total_needed),
                          buffer.end());
}

size_t TopicPacket::MissingBytes() const {
  if (payload_.size() >= full_size_) return 0;
  return full_size_ - payload_.size();
}

void TopicPacket::AddMissingBytes(const std::vector<uint8_t>& more) {
  if (MissingBytes() == 0) {
    remainder_bytes_ = more;  // already complete, all is remainder
    return;
  }
  const size_t before = payload_.size();
  const size_t need = MissingBytes();
  const size_t to_copy = (std::min)(need, more.size());
  payload_.insert(payload_.end(), more.begin(),
                  more.begin() + static_cast<ptrdiff_t>(to_copy));

  if (payload_.size() >= full_size_) {
    // Re-apply alignment trimming when complete.
    if (header_.has_align_padding && !payload_.empty()) {
      const uint8_t pad = payload_.back();
      if (pad <= payload_.size()) {
        real_size_ -= pad;
        payload_.resize(real_size_);
      }
    }
    remainder_bytes_.assign(more.begin() + static_cast<ptrdiff_t>(to_copy),
                            more.end());
  } else {
    remainder_bytes_.clear();
  }
}

// HostInfoPacket ------------------------------------------------------------

std::optional<HostInfoPacket> HostInfoPacket::Parse(
    const std::vector<uint8_t>& bytes) {
  if (bytes.size() < 8) return std::nullopt;
  HostInfoPacket out;
  out.header_0 = LoadLE32(bytes.data());
  out.unk_4 = LoadLE32(bytes.data() + 4);

  out.message_type = static_cast<BuiltinMessageType>(out.header_0 & 0xF);
  out.result = (out.header_0 >> 4) & 0x3FF;
  // stream_size is encoded in bytes (multiple of 4) in bits [12..31]
  out.stream_size = (out.header_0 >> 12) & 0xFFFFC;

  if (out.message_type == BuiltinMessageType::kEcho) {
    if (bytes.size() < 8 + 32) return std::nullopt;
    out.payload.assign(bytes.begin() + 8, bytes.end());
    out.echo_org = LoadLE64(bytes.data() + 8);
    out.echo_recv = LoadLE64(bytes.data() + 16);
    out.echo_xmt = LoadLE64(bytes.data() + 24);
    out.echo_offset = LoadLE64(bytes.data() + 32);
  } else {
    if (bytes.size() < 16) return std::nullopt;
    const uint32_t unk_8 = LoadLE32(bytes.data() + 8);
    const uint32_t len_u64s = LoadLE32(bytes.data() + 12);
    (void)unk_8;
    const size_t payload_bytes = static_cast<size_t>(len_u64s) * 8u;
    if (bytes.size() < 16 + payload_bytes) return std::nullopt;
    out.payload.assign(
        bytes.begin() + 16,
        bytes.begin() + 16 + static_cast<ptrdiff_t>(payload_bytes));
  }
  return out;
}

std::vector<uint8_t> HostInfoPacket::CraftEcho(uint32_t result,
                                               uint32_t echo_id, uint64_t org,
                                               uint64_t recv, uint64_t xmt,
                                               uint64_t offset) {
  std::vector<uint8_t> out;
  // header_0 layout (byte-based stream_size like Python reference)
  const uint32_t message_type =
      static_cast<uint32_t>(BuiltinMessageType::kEcho) & 0xF;
  const uint32_t stream_size_bytes = 0x28;  // 8-byte header + 32-byte echo body
  uint32_t header_0 = 0;
  header_0 |= message_type;
  header_0 |= (result & 0x3FF) << 4;
  header_0 |= ((stream_size_bytes & 0xFFFFC) << 12);

  StoreLE32(header_0, out);
  StoreLE32(echo_id, out);
  StoreLE64(org, out);
  StoreLE64(recv, out);
  StoreLE64(xmt, out);
  StoreLE64(offset, out);
  return out;
}

std::vector<uint8_t> HostInfoPacket::CraftCapnp(
    uint8_t message_type, uint32_t result, uint32_t unk_4,
    const std::vector<uint8_t>& payload) {
  std::vector<uint8_t> out;
  const uint32_t len_u64s = static_cast<uint32_t>(payload.size() / 8u);
  const uint32_t stream_size_bytes = 16u + static_cast<uint32_t>(payload.size());
  uint32_t header_0 = 0;
  header_0 |= (message_type & 0xF);
  header_0 |= (result & 0x3FF) << 4;
  header_0 |= ((stream_size_bytes & 0xFFFFC) << 12);

  StoreLE32(header_0, out);
  StoreLE32(unk_4, out);
  StoreLE32(0, out);         // unk_8
  StoreLE32(len_u64s, out);  // payload length in u64s
  out.insert(out.end(), payload.begin(), payload.end());
  return out;
}

// GenericState --------------------------------------------------------------

void GenericState::Reset() {
  stage = Stage::kSegmentMeta;
  type_idx = 0;
  seg0_size = 0;
  seg1_size = 0;
  seg0.clear();
  seg1.clear();
}

// XrspHost ------------------------------------------------------------------

XrspHost::XrspHost() {
  slice_state_.resize(15);
  start_ns_ = NowNs();
}

XrspHost::~XrspHost() {
  if (dev_) {
    if (iface_number_ >= 0) libusb_release_interface(dev_, iface_number_);
    libusb_close(dev_);
    dev_ = nullptr;
  }
  if (usb_ctx_) {
    libusb_exit(usb_ctx_);
    usb_ctx_ = nullptr;
  }
}

bool XrspHost::InitUsb() {
  if (libusb_init(&usb_ctx_) != 0) {
    fmt::print("libusb_init failed\n");
    return false;
  }

  libusb_device_handle* found =
      libusb_open_device_with_vid_pid(usb_ctx_, kVendorId, kProductId);
  if (!found) {
    fmt::print("device not found (vid=0x{:04x}, pid=0x{:04x})\n", kVendorId, kProductId);
    return false;
  }
  dev_ = found;
  VPrint("usb device found and opened (vid=0x{:04x}, pid=0x{:04x})\n", kVendorId, kProductId);

  if (libusb_set_configuration(dev_, 1) != 0) {
    // Many devices default to configuration 1; ignore errors here.
  }

  libusb_config_descriptor* cfg = nullptr;
  if (libusb_get_active_config_descriptor(libusb_get_device(dev_), &cfg) != 0 ||
      !cfg) {
    VPrint("failed to get active config descriptor\n");
    return false;
  }

  bool claimed = false;
  for (int i = 0; i < cfg->bNumInterfaces && !claimed; ++i) {
    const libusb_interface& intf = cfg->interface[i];
    for (int a = 0; a < intf.num_altsetting && !claimed; ++a) {
      const libusb_interface_descriptor& alt = intf.altsetting[a];
      // Look for one bulk IN and one bulk OUT endpoint.
      uint8_t in_ep = 0, out_ep = 0;
      for (int e = 0; e < alt.bNumEndpoints; ++e) {
        const libusb_endpoint_descriptor& ep = alt.endpoint[e];
        if (!IsBulk(ep)) continue;
        if ((ep.bEndpointAddress & LIBUSB_ENDPOINT_DIR_MASK) ==
            LIBUSB_ENDPOINT_IN) {
          in_ep = ep.bEndpointAddress;
        } else {
          out_ep = ep.bEndpointAddress;
        }
      }
      if (in_ep != 0 && out_ep != 0) {
        if (libusb_claim_interface(dev_, alt.bInterfaceNumber) == 0) {
          iface_number_ = alt.bInterfaceNumber;
          ep_in_ = in_ep;
          ep_out_ = out_ep;
          claimed = true;
          VPrint("successfully claimed interface {} (in: 0x{:02x}, out: 0x{:02x})\n", 
                     alt.bInterfaceNumber, in_ep, out_ep);
          // clear any existing halt conditions on endpoints (parity with python ref)
          int _rc_in = libusb_clear_halt(dev_, ep_in_);
          int _rc_out = libusb_clear_halt(dev_, ep_out_);
          (void)_rc_in; (void)_rc_out;
          break;
        }
      }
    }
  }

  libusb_free_config_descriptor(cfg);

  if (!claimed) {
    VPrint("failed to claim xrsp interface\n");
    return false;
  }

  ResetEcho();
  VPrint("usb initialization complete, sending initial ping\n");
  SendPing();
  return true;
}

bool XrspHost::BulkWrite(const std::vector<uint8_t>& data) {
  if (!dev_) return false;
  int transferred = 0;
  const int rc = libusb_bulk_transfer(
      dev_, ep_out_, const_cast<uint8_t*>(data.data()),
      static_cast<int>(data.size()), &transferred, kUsbTimeoutMs);
  if (rc != 0 || transferred != static_cast<int>(data.size())) {
    VPrint("[usb] bulk write failed rc={} tx={} size={}\n", rc, transferred,
               data.size());
    return false;
  }
  VPrint("[usb] successfully wrote {} bytes\n", transferred);
  return true;
}

bool XrspHost::BulkRead(int size, std::vector<uint8_t>* out) {
  if (!dev_ || size <= 0) return false;
  out->resize(static_cast<size_t>(size));
  int transferred = 0;
  const int rc = libusb_bulk_transfer(dev_, ep_in_, out->data(), size,
                                      &transferred, kUsbTimeoutMs);
  if (rc != 0) {
    if (rc != LIBUSB_ERROR_TIMEOUT) {
      VPrint("bulkread failed: rc={}, transferred={}\n", rc, transferred);
    }
    out->clear();
    return false;
  }
  out->resize(static_cast<size_t>(transferred));
  return true;
}

bool XrspHost::SendToTopic(Topic topic, const std::vector<uint8_t>& bytes) {
  if (bytes.empty()) return true;

  // Break into chunks of up to 0xFFFF inner bytes to match Python sender.
  size_t idx = 0;
  while (idx < bytes.size()) {
    const size_t remain = bytes.size() - idx;
    const size_t chunk = (std::min)(remain, static_cast<size_t>(0xFFFF));
    std::vector<uint8_t> part(
        bytes.begin() + static_cast<ptrdiff_t>(idx),
        bytes.begin() + static_cast<ptrdiff_t>(idx + chunk));
    if (!SendToTopicRaw(topic, part)) return false;
    idx += chunk;
  }
  return true;
}

bool XrspHost::SendToTopicCapnpWrapped(Topic topic, uint32_t idx,
                                       const std::vector<uint8_t>& bytes) {
  std::vector<uint8_t> prelude;
  StoreLE32(idx, prelude);
  StoreLE32(static_cast<uint32_t>(bytes.size() / 8u), prelude);
  if (!SendToTopic(topic, prelude)) return false;
  return SendToTopic(topic, bytes);
}

bool XrspHost::SendToTopicRaw(Topic topic, const std::vector<uint8_t>& bytes) {
  VPrint("[send] sendtotopicraw: topic={}, bytes={}, sequence={}\n", 
             static_cast<int>(topic), bytes.size(), sequence_);
  std::vector<uint8_t> msg(bytes.begin(), bytes.end());

  const size_t real_len = msg.size();
  const size_t align_up_bytes = (((4 + msg.size()) / 4) * 4) - msg.size();
  bool aligned = true;
  if (align_up_bytes != 4 && align_up_bytes != 0) {
    if (align_up_bytes > 1) msg.insert(msg.end(), align_up_bytes - 1, 0xDE);
    msg.push_back(static_cast<uint8_t>((msg.size() + 1) - real_len));
    aligned = false;
  }

  std::vector<uint8_t> pkt_out;
  pkt_out.reserve(8 + msg.size() + 16);
  // Header: <BBHHH (Python) -> we will reconstruct as fields
  // [0]: flags (0x10 aligned, 0x18 unaligned)
  pkt_out.push_back(static_cast<uint8_t>(aligned ? 0x10 : 0x18));
  // [1]: topic index
  pkt_out.push_back(static_cast<uint8_t>(topic));
  // [2..3]: num_words = (len(msg)/4)+1
  StoreLE16(static_cast<uint16_t>((msg.size() / 4) + 1), pkt_out);
  // [4..5]: sequence
  StoreLE16(sequence_, pkt_out);
  // [6..7]: padding
  StoreLE16(0, pkt_out);

  pkt_out.insert(pkt_out.end(), msg.begin(), msg.end());

  // aign to 0x400 boundary by injecting a tiny empty frame if needed
  const size_t to_fill = 0x400 - ((pkt_out.size() + 0x400) & 0x3FF) - 6;
  if (to_fill > 0) {
    // add an empty aligned frame header (<BBHH with no trailing HH?)
    pkt_out.push_back(0x10);
    pkt_out.push_back(0x00);
    StoreLE16(static_cast<uint16_t>((to_fill / 4) + 1), pkt_out);
    StoreLE16(sequence_, pkt_out);  // reuse sequence
    pkt_out.insert(pkt_out.end(), to_fill, 0x00);
  }

  sequence_ = static_cast<uint16_t>((sequence_ + 1) & 0xFFFF);

  VPrint("[send] final packet size: {} bytes\n", pkt_out.size());
  return BulkWrite(pkt_out);
}

uint64_t XrspHost::TimestampNs() const { return NowNs() - start_ns_; }

bool XrspHost::ReadOnce() {
  if (!dev_) return false;

  if (!remainder_bytes_.empty()) {
    fmt::print("processing remainder bytes: {} bytes\n", remainder_bytes_.size());
    TopicPacket pkt(remainder_bytes_);
    if (pkt.MissingBytes() == 0) {
      HandlePacket(pkt);
      remainder_bytes_ = pkt.RemainderBytes();
      return true;
    }
  }

  std::vector<uint8_t> chunk;
  if (!BulkRead(kReadChunk, &chunk)) {
    // Don't spam logs for no data available
    return false;
  }
  if (chunk.empty()) {
    // Don't spam logs for empty reads
    return false;
  }
  
  VPrint("read {} bytes from usb\n", chunk.size());

  if (!working_pkt_.has_value()) {
    working_pkt_.emplace(chunk);
  } else if (working_pkt_->MissingBytes() == 0) {
    HandlePacket(*working_pkt_);
    const std::vector<uint8_t>& remains = working_pkt_->RemainderBytes();
    if (!remains.empty()) {
      if (remains.size() < 8) {
        working_pkt_.reset();
      } else {
        working_pkt_.emplace(remains);
        working_pkt_->AddMissingBytes(chunk);
      }
    } else {
      working_pkt_.emplace(chunk);
    }
  } else {
    working_pkt_->AddMissingBytes(chunk);
  }

  while (working_pkt_.has_value() && working_pkt_->MissingBytes() == 0) {
    HandlePacket(*working_pkt_);
    const std::vector<uint8_t>& remains = working_pkt_->RemainderBytes();
    if (!remains.empty()) {
      if (remains.size() < 8) {
        working_pkt_.reset();
        break;
      } else {
        working_pkt_.emplace(remains);
      }
    } else {
      working_pkt_.reset();
      break;
    }
  }

  return true;
}

void XrspHost::WaitForPairing() {
  const uint64_t start = TimestampNs();
  VPrint("starting pairing process in state: {}\n", static_cast<int>(pairing_state_));
  
  while (pairing_state_ != PairingState::kPaired) {
    ReadOnce();
    if (TimestampNs() - echo_req_sent_ns_ > 1'000'000'000ull &&  // 1s
        (pairing_state_ == PairingState::kPairing ||
         pairing_state_ == PairingState::kWaitSecond)) {
      VPrint("sending ping due to timeout (state: {})\n", static_cast<int>(pairing_state_));
      SendPing();
    }
    // avoid tight spin
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    if (TimestampNs() - start > 30'000'000'000ull) {
      // keep running; headset may require user acceptance before continuing
      VPrint("warning: pairing timeout exceeded 30s, please open headset and try to launch link\n");
    }
  }
  VPrint("pairing completed successfully!\n");
}

void XrspHost::HandlePacket(const TopicPacket& pkt) {
  VPrint("received packet: topic_idx={}, sequence={}, payload_size={}\n", 
             pkt.header().topic_idx, pkt.header().sequence, pkt.payload().size());
  
  if (static_cast<uint8_t>(pkt.header().topic_idx) ==
      static_cast<uint8_t>(Topic::kHostInfoAdv)) {
    HandleHostInfoAdv(pkt);
  } else {
    VPrint("ignoring packet with topic_idx={}\n", pkt.header().topic_idx);
  }
}

void XrspHost::HandleHostInfoAdv(const TopicPacket& pkt) {
  // payload is a builtin header and possibly capnp or echo body
  auto parsed = HostInfoPacket::Parse(pkt.payload());
  if (!parsed.has_value()) {
  VPrint("failed to parse hostinfoadv packet\n");
    return;
  }

  const HostInfoPacket& hi = *parsed;
  VPrint("hostinfoadv: message_type={}, result={}, stream_size={}\n", 
             static_cast<int>(hi.message_type), hi.result, hi.stream_size);
  
  switch (hi.message_type) {
    case BuiltinMessageType::kEcho:
      VPrint("processing echo message\n");
      HandleEcho(hi);
      return;
    default:
      break;
  }

  switch (pairing_state_) {
    case PairingState::kWaitFirst:
      VPrint("in kwaitfirst state, processing message_type={}\n", static_cast<int>(hi.message_type));
      if (hi.message_type == BuiltinMessageType::kInvite) {
        VPrint("received invite, calling initsession()\n");
        InitSession();
      } else if (hi.message_type == BuiltinMessageType::kAck) {
        VPrint("received ack, calling sendcodegen1()\n");
        SendCodegen1();
      } else if (hi.message_type == BuiltinMessageType::kCodeGenerationAck) {
        VPrint("received codegenerationack, calling sendpairing1()\n");
        SendPairing1();
      } else if (hi.message_type == BuiltinMessageType::kPairingAck) {
        VPrint("received pairingack, calling finishpairing1() and transitioning to kwaitsecond\n");
        FinishPairing1();
        pairing_state_ = PairingState::kWaitSecond;
      } else {
        VPrint("unexpected message_type={} in kwaitfirst state\n", static_cast<int>(hi.message_type));
      }
      break;
    case PairingState::kWaitSecond:
    case PairingState::kPairing:
      VPrint("in kwaitsecond/kpairing state, processing message_type={}\n", static_cast<int>(hi.message_type));
      if (hi.message_type == BuiltinMessageType::kInvite) {
        VPrint("received invite, transitioning to kpairing and calling initsession2()\n");
        pairing_state_ = PairingState::kPairing;
        InitSession2();
      } else if (hi.message_type == BuiltinMessageType::kAck) {
        VPrint("received ack, calling sendcodegen2()\n");
        SendCodegen2();
      } else if (hi.message_type == BuiltinMessageType::kCodeGenerationAck) {
        VPrint("received codegenerationack, calling sendpairing2()\n");
        SendPairing2();
      } else if (hi.message_type == BuiltinMessageType::kPairingAck) {
        VPrint("received pairingack, calling finishpairing2() and transitioning to kpaired\n");
        FinishPairing2();
        pairing_state_ = PairingState::kPaired;
      } else {
        VPrint("unexpected message_type={} in kwaitsecond/kpairing state\n", static_cast<int>(hi.message_type));
      }
      break;
    case PairingState::kPaired:
      break;
  }
}

void XrspHost::HandleEcho(const HostInfoPacket& echopkt) {
  if ((echopkt.result & 1u) == kEchoPong) {
    // pong
    echo_req_recv_ns_ = echopkt.echo_recv;  // server recv ns
    echo_resp_sent_ns_ = echopkt.echo_xmt;  // server tx ns
    const uint64_t echo_resp_recv_ns = TimestampNs();
    echo_req_sent_ns_ = TimestampNs();
    // ns_offset = ((srv_recv - cli_send) + (srv_send - cli_recv)) / 2
    const uint64_t cli_send = echo_req_sent_ns_;
    const uint64_t cli_recv = echo_resp_recv_ns;
    const int64_t offs =
        static_cast<int64_t>((static_cast<int64_t>(echo_req_recv_ns_) -
                              static_cast<int64_t>(cli_send)) +
                             (static_cast<int64_t>(echo_resp_sent_ns_) -
                              static_cast<int64_t>(cli_recv)));
    ns_offset_ = static_cast<uint64_t>((offs / 2) & 0xFFFFFFFFFFFFFFFFull);
    if (pairing_state_ == PairingState::kPaired) {
      SendPing();
    }
  } else {
    // ping -> pong
    const uint64_t now = TimestampNs();
    auto pong =
        HostInfoPacket::CraftEcho(kEchoPong, echopkt.unk_4, echopkt.echo_xmt,
                                  echopkt.echo_recv, now, ns_offset_);
    SendToTopic(Topic::kHostInfoAdv, pong);
  }
}

bool XrspHost::SendPing() {
  if (TimestampNs() - echo_req_sent_ns_ < 16'000'000ull)
    return true;  // 16ms throttle
  echo_req_sent_ns_ = TimestampNs();
  std::vector<uint8_t> ping = HostInfoPacket::CraftEcho(
      kEchoPing, echo_idx_, 0, 0, echo_req_sent_ns_, ns_offset_);
  VPrint("sending ping with echo_idx={}\n", echo_idx_);
  SendToTopic(Topic::kHostInfoAdv, ping);
  echo_idx_ += 1;
  return true;
}

void XrspHost::ResetEcho() {
  echo_idx_ = 1;
  ns_offset_ = 0;
  echo_req_sent_ns_ = 0;
  echo_req_recv_ns_ = 0;
  echo_resp_sent_ns_ = 0;
}

// pairing flows

void XrspHost::InitSession() {
  VPrint("[pairing step] initsession() - sending ok response (phase 1)\n");
  // response_ok_payload
  static const uint8_t kPayloadRaw[] = {
      0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x2B, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x03, 0x00,
      0x02, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  std::vector<uint8_t> payload(kPayloadRaw, kPayloadRaw + sizeof(kPayloadRaw));
  const auto ok = HostInfoPacket::CraftCapnp(
      static_cast<uint8_t>(BuiltinMessageType::kOk), 0xC8, 1, payload);
  VPrint("[pairing step] sending ok packet with payload size={}\n", payload.size());
  SendToTopic(Topic::kHostInfoAdv, ok);
}

void XrspHost::SendCodegen1() {
  VPrint("[pairing step] sendcodegen1() - sending codegeneration message (phase 1)\n");
  static const uint8_t kPayloadRaw[] = {
      0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  std::vector<uint8_t> payload(kPayloadRaw, kPayloadRaw + sizeof(kPayloadRaw));
  const auto pkt = HostInfoPacket::CraftCapnp(
      static_cast<uint8_t>(BuiltinMessageType::kCodeGeneration), 0xC8, 1,
      payload);
  VPrint("[pairing step] sending codegeneration packet with payload size={}\n", payload.size());
  SendToTopic(Topic::kHostInfoAdv, pkt);
}

void XrspHost::SendPairing1() {
  VPrint("[pairing step] sendpairing1() - sending pairing message (phase 1)\n");
  static const uint8_t kPayloadRaw[] = {0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
                                        0x00, 0x00, 0x01, 0x00, 0x01, 0x00,
                                        0x00, 0x00, 0x00, 0x00};
  std::vector<uint8_t> payload(kPayloadRaw, kPayloadRaw + sizeof(kPayloadRaw));
  const auto pkt = HostInfoPacket::CraftCapnp(
      static_cast<uint8_t>(BuiltinMessageType::kPairing), 0xC8, 1, payload);
  VPrint("[pairing step] sending pairing packet with payload size={}\n", payload.size());
  SendToTopic(Topic::kHostInfoAdv, pkt);
}

void XrspHost::FinishPairing1() {
  VPrint("[pairing step] finishpairing1() - completing phase 1 pairing\n");
  // send ping and a small command on TOPIC_VIDEO wrapped
  VPrint("[pairing step] sending ping in finishpairing1\n");
  SendPing();
  static const uint8_t kVideoCmdRaw[] = {0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
                                         0x00, 0x00, 0x02, 0x00, 0x00, 0x00};
  std::vector<uint8_t> body(kVideoCmdRaw, kVideoCmdRaw + sizeof(kVideoCmdRaw));
  VPrint("[pairing step] sending video command on topic::kvideo\n");
  SendToTopicCapnpWrapped(Topic::kVideo, 0, body);
}

void XrspHost::InitSession2() {
  VPrint("[pairing step] initsession2() - starting phase 2 pairing\n");
  ResetEcho();
  VPrint("[pairing step] echo state reset for phase 2\n");
  // response_ok_2_payload
  static const uint8_t kPayloadRaw[] = {
      0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x03, 0x00, 0x03, 0x00, 0x01, 0x00,
      0x1F, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x48, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x09, 0x00, 0x00, 0x00, 0x1B, 0x00, 0x00, 0x00,
      0x01, 0x00, 0x00, 0x00, 0x2A, 0x00, 0x00, 0x00, 'U',  'S',  'B',  '3',
      0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x03, 0x00, 0x02, 0x00, 0x00, 0x00};
  std::vector<uint8_t> payload(kPayloadRaw, kPayloadRaw + sizeof(kPayloadRaw));
  const auto ok2 = HostInfoPacket::CraftCapnp(
      static_cast<uint8_t>(BuiltinMessageType::kOk), 0xC8, 1, payload);
  VPrint("[pairing step] sending ok2 packet with payload size={} (includes 'usb3' string)\n", payload.size());
  SendToTopic(Topic::kHostInfoAdv, ok2);
}

void XrspHost::SendCodegen2() {
  VPrint("[pairing step] sendcodegen2() - sending codegeneration message (phase 2)\n");
  static const uint8_t kPayloadRaw[] = {
      0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x03, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  std::vector<uint8_t> payload(kPayloadRaw, kPayloadRaw + sizeof(kPayloadRaw));
  const auto pkt = HostInfoPacket::CraftCapnp(
      static_cast<uint8_t>(BuiltinMessageType::kCodeGeneration), 0xC8, 1,
      payload);
  VPrint("[pairing step] sending codegeneration2 packet with payload size={}\n", payload.size());
  SendToTopic(Topic::kHostInfoAdv, pkt);
}

void XrspHost::SendPairing2() {
  VPrint("[pairing step] sendpairing2() - sending pairing message (phase 2)\n");
  static const uint8_t kPayloadRaw[] = {0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
                                        0x00, 0x00, 0x03, 0x00, 0x00, 0x00,
                                        0x00, 0x00, 0x00, 0x00};
  std::vector<uint8_t> payload(kPayloadRaw, kPayloadRaw + sizeof(kPayloadRaw));
  const auto pkt = HostInfoPacket::CraftCapnp(
      static_cast<uint8_t>(BuiltinMessageType::kPairing), 0xC8, 1, payload);
  VPrint("[pairing step] sending pairing2 packet with payload size={}\n", payload.size());
  SendToTopic(Topic::kHostInfoAdv, pkt);
}

void XrspHost::FinishPairing2() {
  VPrint("[pairing step] finishpairing2() - pairing process completed!\n");
  // nothing extra here
}

}  // namespace xrsp
