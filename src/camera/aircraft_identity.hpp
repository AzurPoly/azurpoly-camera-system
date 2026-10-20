/*
 * Defines the public aircraft identity and flight-session state used to recognize the Rafale. Request IDs
 * and session changes keep delayed SimConnect replies from being accepted as current aircraft data.
 */

#pragma once
#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include "../shared/configuration.hpp"

namespace azp::native_camera {
struct AircraftIdentitySample {
  std::array<char, 256> type{};
  std::array<char, 260> path{};
  std::uint64_t sample_ms = 0;
  bool rafale = false;
  bool fresh = false;
};
/*
 * Public SimConnect lifecycle notifications. The first Sim event reports the current state;
 * subscribing/reconnecting must not manufacture a new flight.
 */
class AircraftSessionLifecycle {
 public:
  static constexpr std::uint32_t SimEvent = 100, AircraftEvent = 101, FlightEvent = 102;
  bool accept(const void* raw, std::uint32_t bytes) noexcept;
  void changed() noexcept;
  std::uint64_t epoch() const noexcept { return epoch_; }
  bool running() const noexcept { return !known_ || running_; }

 private:
  std::uint64_t epoch_ = 0;
  bool known_ = false, running_ = false;
};
/*
 * Cached public metadata only; owns no simulator objects and makes no API calls.
 */
class AircraftIdentityCache {
 public:
  /*
   * Both responses must belong to this poll. Incomplete/late polls cannot combine the previous aircraft's
   * path with the next aircraft's type.
   */
  void begin_request(std::uint32_t type_request, std::uint32_t path_request) noexcept;
  bool accept(const void* raw, std::uint32_t bytes, std::uint64_t now) noexcept;
  AircraftIdentitySample sample(std::uint64_t now) const noexcept;

 private:
  AircraftIdentitySample value_, pending_;
  std::uint64_t type_ms_ = 0, path_ms_ = 0;
  std::uint64_t pending_type_ms_ = 0, pending_path_ms_ = 0;
  std::uint32_t type_request_ = 5, path_request_ = 6;
};
}  /* namespace azp::native_camera */
