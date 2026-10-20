/*
 * Converts aircraft/world coordinates and validated WASM publications into a mounted camera pose. It handles
 * radians versus degrees, metres and basis vectors, and refuses stale or incomplete publications rather than
 * aiming with mixed data.
 */

#include "pose.hpp"

namespace azp::native_camera::mount_detail {
bool finite(const Vector3& v) noexcept {
  return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

double dot(const Vector3& a, const Vector3& b) noexcept {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

Vector3 cross(const Vector3& a, const Vector3& b) noexcept {
  return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

Vector3 rotate(const BodyPose& body, const Vector3& v) noexcept {
  Vector3 result{};
  for (unsigned i = 0; i < result.size(); ++i)
    result[i] = body.right[i] * v[0] + body.up[i] * v[1] + body.forward[i] * v[2];
  return result;
}

}  /* namespace azp::native_camera::mount_detail */

namespace azp::native_camera {
/*
 * Checks finite position and an orthonormal right/up/forward basis. Coordinate-system handedness is not
 * assumed; the basis must be geometrically consistent.
 *
 * Semantic right/up/forward can have either handedness in the destination coordinate system. Reject
 * nonorthogonal bases, not an ECEF axis convention.
 */
bool valid_body_pose(const BodyPose& body) noexcept {
  using namespace mount_detail;
  if (!finite(body.origin) || !finite(body.right) || !finite(body.up) || !finite(body.forward))
    return false;
  constexpr double tolerance = 0.001;
  return std::abs(dot(body.right, body.right) - 1) <= tolerance && std::abs(dot(body.up, body.up) - 1) <= tolerance &&
         std::abs(dot(body.forward, body.forward) - 1) <= tolerance && std::abs(dot(body.right, body.up)) <= tolerance &&
         std::abs(dot(body.right, body.forward)) <= tolerance && std::abs(dot(body.up, body.forward)) <= tolerance &&
         std::abs(std::abs(dot(cross(body.right, body.up), body.forward)) - 1) <= tolerance;
}

}  /* namespace azp::native_camera */

namespace azp::native_camera::body_math {
bool valid(const Telemetry& p) noexcept {
  const double values[]{p.latitude, p.longitude, p.altitude, p.pitch, p.bank, p.heading};
  for (double x : values)
    if (!std::isfinite(x))
      return false;
  return std::abs(p.latitude) <= 90 && std::abs(p.longitude) <= 180 && p.altitude >= -1000 && p.altitude <= 100000 &&
         std::abs(p.pitch) <= 180 && std::abs(p.bank) <= 180 && std::abs(p.heading) <= 720;
}

/*
 * Converts latitude/longitude in degrees and altitude in metres to Earth-centred Cartesian metres on the
 * ellipsoid used by this module.
 */
Vector3 ecef(double lat, double lon, double height) noexcept {
  lat *= radians;
  lon *= radians;
  const double s = std::sin(lat), c = std::cos(lat), n = a / std::sqrt(1 - e2 * s * s);
  return {(n + height) * c * std::cos(lon), (n + height) * c * std::sin(lon), (n * (1 - e2) + height) * s};
}

/*
 * Converts Earth-centred Cartesian metres to geodetic latitude/longitude and altitude with a bounded
 * ellipsoid iteration. Degenerate or non-finite inputs are refused.
 */
bool geodetic(const Vector3& p, Vector3& result) noexcept {
  if (!mount_detail::finite(p))
    return false;
  const double xy = std::hypot(p[0], p[1]), radius = std::hypot(xy, p[2]);
  if (radius < 6300000 || radius > 6500000 || xy < 1)
    return false;
  double lat = std::atan2(p[2], xy * (1 - e2)), h = 0;
  for (unsigned i = 0; i < 10; ++i) {
    const double s = std::sin(lat), n = a / std::sqrt(1 - e2 * s * s);
    h = xy / std::cos(lat) - n;
    lat = std::atan2(p[2], xy * (1 - e2 * n / (n + h)));
  }
  const double s = std::sin(lat), n = a / std::sqrt(1 - e2 * s * s);
  h = xy / std::cos(lat) - n;
  result = {lat / radians, std::atan2(p[1], p[0]) / radians, h};
  return mount_detail::finite(result);
}

/*
 * Builds the aircraft's world position and right/up/forward axes from geodetic position and
 * pitch/bank/heading, using the module's angular conventions.
 */
BodyPose body(const Telemetry& p, double vertical_correction) noexcept {
  BodyPose out;
  out.origin = ecef(p.latitude, p.longitude, p.altitude + vertical_correction);
  const double lat = p.latitude * radians, lon = p.longitude * radians, h = p.heading * radians, pitch = p.pitch * radians,
               bank = p.bank * radians;
  const Vector3 east{-std::sin(lon), std::cos(lon), 0};
  const Vector3 north{-std::sin(lat) * std::cos(lon), -std::sin(lat) * std::sin(lon), std::cos(lat)};
  const Vector3 up{std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat)};
  for (unsigned i = 0; i < 3; ++i) {
    const double f = north[i] * std::cos(h) + east[i] * std::sin(h);
    const double r = east[i] * std::cos(h) - north[i] * std::sin(h);
    const double u = up[i] * std::cos(pitch) + f * std::sin(pitch);
    out.forward[i] = f * std::cos(pitch) - up[i] * std::sin(pitch);
    out.right[i] = r * std::cos(bank) + u * std::sin(bank);
    out.up[i] = u * std::cos(bank) - r * std::sin(bank);
  }
  return out;
}

double distance(const Vector3& x, const Vector3& y) noexcept {
  return std::hypot(std::hypot(x[0] - y[0], x[1] - y[1]), x[2] - y[2]);
}

}  /* namespace azp::native_camera::body_math */

namespace azp::native_camera {
/*
 * Validates the complete WASM FLOAT64 publication, including version, sequence and finite/range checks. An
 * unfinished publication cannot renew the last valid sample's freshness.
 *
 * An in-progress publication does not renew its age.
 */
bool ImaCameraCache::accept(const void* packet, std::uint32_t bytes, std::uint64_t now) noexcept {
  if (!packet || bytes != 40 + sizeof(double) * std::size(ImaCameraVariables) || !now)
    return false;
  std::array<std::uint32_t, 10> header{};
  std::memcpy(header.data(), packet, sizeof(header));
  if (header[0] != bytes || header[2] != 8 || header[3] != 9 || header[5] != 9 || header[6] != 0 ||
      header[9] != std::size(ImaCameraVariables))
    return false;
  std::array<double, std::size(ImaCameraVariables)> value{};
  std::memcpy(value.data(), static_cast<const unsigned char*>(packet) + 40, sizeof(value));
  for (auto number : value)
    if (!std::isfinite(number)) {
      sample_ = {};
      return true;
    }
  if (value[0] <= 0 || value[0] > 9007199254740990.0 || value[0] != value.back() || std::fmod(value[0], 2) != 0)
    return true;
  if (std::abs(value[1]) > 90 || std::abs(value[2]) > 720 || value[3] < 0 || value[3] >= 360 || value[4] <= 0 || value[4] >= 180 ||
      (value[5] != 0 && value[5] != 1) || value[9] != 1) {
    sample_ = {};
    return true;
  }
  if (value[0] != sequence_) {
    sequence_ = value[0];
    sample_ = {true,
               value[5] == 1,
               value[1],
               value[2],
               value[3],
               value[4],
               {value[6], value[7], value[8]},
               now,
               value[9] == 1,
               {value[10], value[11], value[12]}};
  }
  return true;
}

ImaCameraSample ImaCameraCache::sample(std::uint64_t now) const noexcept {
  auto result = sample_;
  result.valid = result.valid && now >= result.sample_ms && now - result.sample_ms <= 1000;
  return result;
}

/*
 * Combines the aircraft basis with the published mount and orientation, converting angular units before
 * forming direction/up vectors. A valid no-target publication uses the forward view; stale or malformed
 * guidance produces no pose.
 */
bool make_ima_camera_pose(const BodyPose& body, const ImaCameraSample& ima, MountedPose& output) noexcept {
  output = {};
  if (!valid_body_pose(body))
    return false;
  output.up = body.up;
  if (!ima.valid || !ima.local_mount)
    return false;
  if (!std::isfinite(ima.fov) || ima.fov <= 0 || ima.fov >= 180 || !mount_detail::finite(ima.mount_m))
    return false;
  output.fov = static_cast<float>(ima.fov * body_math::radians);
  Vector3 direction = body.forward;
  if (ima.has_target) {
    Vector3 location{};
    if (!body_math::geodetic(body.origin, location))
      return false;
    const auto view = body_math::body({location[0], location[1], location[2], ima.pitch, ima.bank, ima.heading}, 0);
    if (!valid_body_pose(view))
      return false;
    direction = view.forward;
    output.up = view.up;
  }
  const auto offset = mount_detail::rotate(body, ima.mount_m);
  for (unsigned i = 0; i < 3; ++i) {
    output.position[i] = body.origin[i] + offset[i];
    output.target[i] = output.position[i] + direction[i];
  }
  if (!mount_detail::finite(output.position) || !mount_detail::finite(output.target))
    return false;
  Vector3 represented_direction{};
  for (unsigned i = 0; i < 3; ++i)
    represented_direction[i] = output.target[i] - output.position[i] - direction[i];
  return mount_detail::dot(represented_direction, represented_direction) <= 1e-8;
}

}  /* namespace azp::native_camera */
