/*
 * Declares zoom detail: what AZP adjusts so that its zoomed image shows distant objects as a close view would.
 *
 * - LOD. MSFS gives every model piece one LOD for all views, from the viewpoint nearest to it, whatever that
 *   view's aim (docs/investigations/model-lod.md). The AZP camera is 2.3 m from the pilot's eye: when the eye is
 *   nearer to a piece, the AZP zoom was ignored; when the AZP camera is nearer, its zoom raised every piece on
 *   its side, in its image or not. zoom_detail.cpp filters the viewpoints the engine's selection sees: a piece
 *   inside the AZP image is judged from the AZP camera alone, a piece outside it from the other views alone.
 *   The engine's own selection does the rest (minimum sizes, LOD limits, forced and hidden pieces).
 * - Fade. The renderer fades each mesh out beyond about 250 times its radius, measured from the view's camera
 *   and multiplied by a per-view factor that ignores the FOV (docs/investigations/mesh-fade.md). The AZP view's
 *   factor is scaled by its zoom, so a mesh fades out at the same size in the image as at zoom 1.
 *
 * inside_image is pure (tests/zoom_detail_test.cpp).
 */

#pragma once
#include <cmath>
#include <cstdint>
#include <string>
#include "pose.hpp"

namespace azp::native_camera {

/*
 * Control thread. The first enable resolves the engine contract and installs the LOD redirect; later calls
 * only switch it. Disabled, every selection sees the engine's viewpoint list.
 */
void set_zoom_detail_enabled(bool enabled) noexcept;
/*
 * From the camera-update callback, every frame: the AZP camera node's world position (the engine's viewpoint
 * position), its unit look direction and its FOV in radians. A publication older than 250 ms is ignored.
 */
void publish_zoom_detail_view(const Vector3& position, const Vector3& forward, double fov) noexcept;
/*
 * From the camera observer, on the owned view right after the native view update has rewritten its fade
 * factor: scales that factor by tan(FOV / 2) / tan(15 degrees) (`fov` in radians). Zoom 1 and wider keep the
 * engine's factor.
 */
void apply_zoom_fade(std::uint64_t view_address, double fov) noexcept;
/*
 * The installation state, for the bridge log.
 */
std::string zoom_detail_message();

namespace zoom_detail {

/*
 * Whether a sphere (world center, radius in metres) can appear in a square image seen from `eye` along the
 * unit `forward`, with tan(FOV / 2) = tan_half: within the corner angle plus the sphere's angular radius.
 */
inline bool inside_image(const Vector3& eye, const Vector3& forward, double tan_half, const Vector3& center, double radius) noexcept {
  const Vector3 offset{center[0] - eye[0], center[1] - eye[1], center[2] - eye[2]};
  const auto distance = std::sqrt(offset[0] * offset[0] + offset[1] * offset[1] + offset[2] * offset[2]);
  if (!(distance > radius) || !(tan_half > 0))
    return std::isfinite(distance);
  const auto angle = std::atan(std::sqrt(2.0) * tan_half) + std::asin(radius / distance);
  const auto along = (offset[0] * forward[0] + offset[1] * forward[1] + offset[2] * forward[2]) / distance;
  return angle >= 1.5707963267948966 ? along > -1 : along >= std::cos(angle);
}

}  /* namespace zoom_detail */
}  /* namespace azp::native_camera */
