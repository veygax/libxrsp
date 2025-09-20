#include <chrono>
#include <cstdint>
#include <iostream>
#include <thread>

#include "xrsp.h"

int main() {
  xrsp::XrspHost host;
  if (!host.InitUsb()) {
    std::cout << "Failed to initialize USB" << std::endl;
    return 1;
  }

  std::cout << "Waiting for pairing..." << std::endl;
  host.WaitForPairing();
  std::cout << "Paired." << std::endl;

  // Idle read loop for a few seconds.
  const uint64_t start = host.TimestampNs();
  while (host.TimestampNs() - start < 5'000'000'000ull) {
    host.ReadOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  return 0;
}
