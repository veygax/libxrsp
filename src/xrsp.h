#ifndef LIBXRSP_SRC_XRSP_H_
#define LIBXRSP_SRC_XRSP_H_

#include <cstdint>
#include <optional>
#include <vector>

struct libusb_context;
struct libusb_device_handle;

namespace xrsp {

// topics (subset)
enum class Topic : uint8_t {
  kAui4aAdv = 0x00,
  kHostInfoAdv = 0x01,
  kCommand = 0x02,
  kPose = 0x03,
  kMesh = 0x04,
  kVideo = 0x05,
  kAudio = 0x06,
  kHaptic = 0x07,
  kHands = 0x08,
  kSkeleton = 0x09,
  kSlice0 = 0x0A,
  kSlice1 = 0x0B,
  kSlice2 = 0x0C,
  kSlice3 = 0x0D,
  kSlice4 = 0x0E,
  kSlice5 = 0x0F,
  kSlice6 = 0x10,
  kSlice7 = 0x11,
  kSlice8 = 0x12,
  kSlice9 = 0x13,
  kSlice10 = 0x14,
  kSlice11 = 0x15,
  kSlice12 = 0x16,
  kSlice13 = 0x17,
  kSlice14 = 0x18,
  kSlice15 = 0x19,
  kAudioControl = 0x1A,
  kUserSettingsSync = 0x1B,
  kInputControl = 0x1C,
  kAsw = 0x1D,
  kBody = 0x1E,
  kRuntimeIpc = 0x1F,
  kCameraStream = 0x20,
  kLogging = 0x21,
};

// builtin message type for hostinfo-adv
enum class BuiltinMessageType : uint8_t {
  kPairingAck = 0x0,
  kInvite = 0x1,
  kOk = 0x2,
  kAck = 0x3,
  kError = 0x4,
  kBye = 0x5,
  kEcho = 0x6,
  kPairing = 0x7,
  kCodeGeneration = 0x9,
  kCodeGenerationAck = 0xA,
  kReserved = 0xF,
};

// echo result bit (0 = ping, 1 = pong)
constexpr uint32_t kEchoPing = 0;
constexpr uint32_t kEchoPong = 1;

// parsed XRSP packet header info
struct PacketHeader {
  // original 16-bit topic+flags word
  uint16_t topic_raw = 0;
  // unpacked fields
  uint8_t version_low3 = 0;             // bits 0..2
  bool has_align_padding = false;       // bit 3
  bool packet_version_is_internal = 0;  // bit 4
  bool packet_version_number = 0;       // bit 5
  uint8_t topic_idx = 0;                // bits 8..13
  uint8_t unk_14_15 = 0;                // bits 14..15

  uint16_t num_words = 0;  // includes header word, in 32-bit words
  uint16_t sequence = 0;
};

// topic packet view and reassembly helper
class TopicPacket {
 public:
  TopicPacket() = default;
  explicit TopicPacket(const std::vector<uint8_t>& buffer);

  // returns how many payload bytes are still missing for a complete packet
  size_t MissingBytes() const;

  // append additional bytes to complete the payload
  void AddMissingBytes(const std::vector<uint8_t>& more);

  // remaining bytes after the packet ends (may hold the start of next packet)
  const std::vector<uint8_t>& RemainderBytes() const {
    return remainder_bytes_;
  }

  // accessors
  const PacketHeader& header() const { return header_; }
  const std::vector<uint8_t>& payload() const { return payload_; }

 private:
  void ParseHeader(const std::vector<uint8_t>& buffer);
  void FinalizePayloadViews(const std::vector<uint8_t>& buffer);

  PacketHeader header_{};
  size_t real_size_ = 0;  // payload bytes (after removing align padding)
  size_t full_size_ = 0;  // payload bytes before removing align padding

  std::vector<uint8_t> payload_;
  std::vector<uint8_t> remainder_bytes_;
};

// minimal parser for HostInfo (builtin) packets used for echo/pairing
struct HostInfoPacket {
  // raw header fields
  uint32_t header_0 = 0;  // packed fields
  uint32_t unk_4 = 0;     // message id / stream version

  // decoded header_0
  BuiltinMessageType message_type = BuiltinMessageType::kReserved;
  uint32_t result = 0;       // 10-bit status (opaque)
  uint32_t stream_size = 0;  // size in bytes from spec words*4, includes header

  // echo-only fields
  uint64_t echo_org = 0;
  uint64_t echo_recv = 0;
  uint64_t echo_xmt = 0;
  uint64_t echo_offset = 0;

  // generic payload for non-echo messages (raw bytes after builtin header)
  std::vector<uint8_t> payload;

  // parse from raw bytes beginning at builtin header
  static std::optional<HostInfoPacket> Parse(const std::vector<uint8_t>& bytes);

  // craft echo packet bytes (PING or PONG)
  static std::vector<uint8_t> CraftEcho(uint32_t result, uint32_t echo_id,
                                        uint64_t org, uint64_t recv,
                                        uint64_t xmt, uint64_t offset);

  // craft generic builtin with capnp-like framed payload (len is in u64s)
  static std::vector<uint8_t> CraftCapnp(uint8_t message_type, uint32_t result,
                                         uint32_t unk_4,
                                         const std::vector<uint8_t>& payload);
};

struct GenericState {
  enum class Stage { kSegmentMeta, kSegmentRead, kExtRead };

  Stage stage = Stage::kSegmentMeta;
  uint32_t type_idx = 0;
  size_t seg0_size = 0;
  size_t seg1_size = 0;
  std::vector<uint8_t> seg0;
  std::vector<uint8_t> seg1;

  void Reset();
};

struct CameraStreamState {
  bool has_meta = false;
  bool seq_collecting = false;
  uint32_t which = 0;
  size_t seq_seg0_size = 0;
  size_t seq_seg1_size = 0;
  size_t seq_seg2_size = 0;
  std::vector<uint8_t> seq_seg0;
  std::vector<uint8_t> seq_seg1;
  std::vector<uint8_t> seq_seg2;
};

enum class PairingState {
  kWaitFirst = 0,
  kWaitSecond = 1,
  kPairing = 2,
  kPaired = 3,
};

class XrspHost {
 public:
  XrspHost();
  ~XrspHost();

  XrspHost(const XrspHost&) = delete;
  XrspHost& operator=(const XrspHost&) = delete;

  // initialize libusb and locate the XRSP interface (claims interface)
  bool InitUsb();

  // read and process available packets once. returns true on success
  bool ReadOnce();

  // block until paired (runs ReadOnce internally until state is Paired)
  void WaitForPairing();

  // utilities to send
  bool SendToTopic(Topic topic, const std::vector<uint8_t>& bytes);
  bool SendToTopicCapnpWrapped(Topic topic, uint32_t idx,
                               const std::vector<uint8_t>& bytes);

  uint64_t TimestampNs() const;

 private:
  void HandlePacket(const TopicPacket& pkt);
  void HandleHostInfoAdv(const TopicPacket& pkt);
  void HandleEcho(const HostInfoPacket& echo_pkt);

  bool SendPing();
  void ResetEcho();

  bool BulkWrite(const std::vector<uint8_t>& data);
  bool BulkRead(int size, std::vector<uint8_t>* out);

  bool SendToTopicRaw(Topic topic, const std::vector<uint8_t>& bytes);

  void InitSession();
  void SendCodegen1();
  void SendPairing1();
  void FinishPairing1();
  void InitSession2();
  void SendCodegen2();
  void SendPairing2();
  void FinishPairing2();

  libusb_context* usb_ctx_ = nullptr;
  libusb_device_handle* dev_ = nullptr;
  int iface_number_ = -1;  // claimed interface
  uint8_t ep_in_ = 0;      // endpoint addresses
  uint8_t ep_out_ = 0;

  uint16_t sequence_ = 0;
  PairingState pairing_state_ = PairingState::kWaitFirst;

  uint32_t echo_idx_ = 1;
  uint64_t ns_offset_ = 0;
  uint64_t echo_req_sent_ns_ = 0;   // client ns
  uint64_t echo_req_recv_ns_ = 0;   // server ns
  uint64_t echo_resp_sent_ns_ = 0;  // server ns
  uint64_t start_ns_ = 0;

  // read assembly
  std::vector<uint8_t> remainder_bytes_;
  std::optional<TopicPacket> working_pkt_;

  // per-topic states (subset)
  CameraStreamState camera_state_{};
  GenericState pose_state_{};
  GenericState audio_state_{};
  GenericState logging_state_{};
  GenericState ipc_state_{};
  std::vector<GenericState> slice_state_;  // 15 entries
};

}  // namespace xrsp

#endif  // LIBXRSP_SRC_XRSP_H_
