/*
 * Defines camera pose math, WASM publication fields and the guidance cache. Position offsets are relative to
 * the aircraft; a valid publication without a target produces the forward view, while an invalid publisher
 * cannot supply a new pose.
 */

#pragma once
#include <array>
#include <cmath>
#include <cstring>

namespace azp::native_camera {

using Vector3 = std::array<double, 3>;

/*
 * The caller establishes the aircraft datum, world units and current body orientation. This helper cannot
 * derive them from a pilot camera pose.
 */
struct BodyPose {
  Vector3 origin{};
  Vector3 right{1, 0, 0};
  Vector3 up{0, 1, 0};
  Vector3 forward{0, 0, 1};
};

struct MountedPose {
  Vector3 position{};
  Vector3 target{};
  Vector3 up{};
  float fov = 0;
};

namespace mount_detail {
bool finite(const Vector3& v) noexcept;
double dot(const Vector3& a, const Vector3& b) noexcept;
Vector3 cross(const Vector3& a, const Vector3& b) noexcept;
Vector3 rotate(const BodyPose& body, const Vector3& v) noexcept;
}  /* namespace mount_detail */

bool valid_body_pose(const BodyPose& body) noexcept;

}  /* namespace azp::native_camera */

namespace azp::native_camera::body_math {
inline constexpr double pi = 3.14159265358979323846;
inline constexpr double radians = pi / 180.0;
inline constexpr double a = 6378137.0;
inline constexpr double e2 = 6.6943799901413165e-3;
struct Telemetry {
  double latitude = 0, longitude = 0, altitude = 0, pitch = 0, bank = 0, heading = 0;
};
bool valid(const Telemetry& p) noexcept;
Vector3 ecef(double lat, double lon, double height) noexcept;
bool geodetic(const Vector3& p, Vector3& result) noexcept;
BodyPose body(const Telemetry& p, double vertical_correction) noexcept;
double distance(const Vector3& x, const Vector3& y) noexcept;
}  /* namespace azp::native_camera::body_math */

namespace azp::native_camera {
/*
 * The WASM gauge computes world PBH from IMA target Lvars (pitch positive down), FOV in degrees and a
 * versioned body-local mount, within an odd/even publication. The legacy ECEF fields remain in the wire
 * layout; placement requires mount version 1.
 */
inline constexpr const char* ImaCameraVariables[]{
    "L:1:AZP_RAF_WASM_CAMERA_SEQUENCE",        "L:1:AZP_RAF_WASM_CAMERA_PITCH",         "L:1:AZP_RAF_WASM_CAMERA_BANK",
    "L:1:AZP_RAF_WASM_CAMERA_HEADING",         "L:1:AZP_RAF_WASM_CAMERA_FOV",           "L:1:AZP_RAF_WASM_CAMERA_HAS_TARGET",
    "L:1:AZP_RAF_WASM_CAMERA_POSITION_X_M",    "L:1:AZP_RAF_WASM_CAMERA_POSITION_Y_M",  "L:1:AZP_RAF_WASM_CAMERA_POSITION_Z_M",
    "L:1:AZP_RAF_WASM_CAMERA_MOUNT_VERSION",   "L:1:AZP_RAF_WASM_CAMERA_MOUNT_RIGHT_M", "L:1:AZP_RAF_WASM_CAMERA_MOUNT_UP_M",
    "L:1:AZP_RAF_WASM_CAMERA_MOUNT_FORWARD_M", "L:1:AZP_RAF_WASM_CAMERA_SEQUENCE"};
struct ImaCameraSample {
  bool valid = false, has_target = false;
  double pitch = 0, bank = 0, heading = 0, fov = 40;
  Vector3 position_offset{};
  std::uint64_t sample_ms = 0;
  bool local_mount = false;
  Vector3 mount_m{};
};

class ImaCameraCache {
 public:
  bool accept(const void* packet, std::uint32_t bytes, std::uint64_t now) noexcept;
  ImaCameraSample sample(std::uint64_t now) const noexcept;

 private:
  double sequence_ = 0;
  ImaCameraSample sample_{};
};

/*
 * The WASM gauge owns placement and orientation; the scene supplies the model origin. Missing/stale
 * publication pauses capture; without a target, look straight ahead.
 */
bool make_ima_camera_pose(const BodyPose& body, const ImaCameraSample& ima, MountedPose& output) noexcept;
}  /* namespace azp::native_camera */
