/*
 * Describes the aircraft transform read from the live scene hierarchy and its inspection result. This
 * transform is the attachment reference; public telemetry is used only to check its plausibility and flight
 * context.
 */

#pragma once
#include <cstring>
#include "../camera/pose.hpp"
#include "camera_layout.hpp"
#include "memory.hpp"

namespace azp::native_camera {

struct AircraftScenePose {
  bool complete = false;
  const char* error = "unavailable";
  BodyPose pose{};
  std::uint32_t read_bytes = 0;
  /*
   * Borrowed only for this verified engine update; no reference is acquired.
   */
  std::uint64_t node = 0;
};

/*
 * The caller supplies a freshly verified active user/controller in the current observer, under the complete
 * code contract including the +336 handle accessor. Never retains a borrowed pointer, acquires a reference,
 * or writes the scene. 1.8.16.0 observations: controller+336 generation handle and +464 alias name the same
 * model Node; its +296 matrix contains left/up/forward/origin rows. Attached model type/vtable and a
 * complete reread are required. These checks do not independently identify an aircraft: graph/session
 * ownership remains the caller's responsibility. Public pose is a separate plausibility guard.
 */
AircraftScenePose inspect_aircraft_scene_pose(engine_camera::MemoryReader& reader,
                                              std::uint64_t verified_user,
                                              std::uint64_t image_base,
                                              const CameraImageLayout& layout) noexcept;

/*
 * This only rejects a wrong/stale aircraft reference. It never blends the asynchronous public pose into the
 * scene pose used to position cameras.
 */
bool scene_body_matches_public(const BodyPose& scene, const BodyPose& public_pose, double maximum_distance = 25) noexcept;

}  /* namespace azp::native_camera */
