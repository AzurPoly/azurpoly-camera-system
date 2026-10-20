/*
 * Declares attachment setup, preparation and release. Callers must supply currently validated
 * camera/aircraft identities; the implementation owns hook state and restores a parent only while that
 * relationship is still proven.
 */

#pragma once
#include "../engine/aircraft_scene_pose.hpp"
#include "../engine/contracts/camera_attachment_contract.hpp"
#include "../engine/view.hpp"
#include "pose.hpp"

namespace azp::native_camera {

/*
 * Installs the camera-update hook from the resolved attachment contract (engine/engine_api.cpp).
 * Attachment/release then run only in the manager observer with fresh owned-view and aircraft snapshots.
 */
bool initialize_camera_attachment(const CameraAttachmentContract& contract,
                                  const discovery::Inventory& image,
                                  std::uint64_t base,
                                  const CameraContract& camera,
                                  std::string& error);
bool prepare_camera_attachment(const engine_camera::OwnedViewSnapshot& view,
                               const AircraftScenePose& aircraft,
                               const ImaCameraSample& ima,
                               bool& attached,
                               std::string& error);
bool release_camera_attachment(const engine_camera::OwnedViewSnapshot& view, std::string& error);
/*
 * Clear weak tracking only after the controller has retired every owned entry. Never detaches or
 * dereferences a node whose owned entry is no longer present.
 */
void forget_camera_attachment();
}  /* namespace azp::native_camera */
