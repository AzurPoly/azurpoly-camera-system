/*
 * Declares the rigid-attachment contract: the native node-parenting operations and the camera-update
 * callback slot used to keep AZP's camera fixed to the Rafale. A valid result supplies verified RVAs to
 * camera/camera_attachment.cpp; resolving it installs nothing.
 */

#pragma once
#include <cstdint>
#include <string>
#include "camera_contract.hpp"

namespace azp::native_camera {

struct CameraAttachmentContract {
  bool valid = false;
  std::string error;
  std::uint32_t attach = 0, detach = 0, update = 0, vtable = 0;
};
CameraAttachmentContract resolve_camera_attachment_contract(discovery::ImageReader& reader,
                                                            const discovery::Inventory& image,
                                                            std::uint64_t base,
                                                            const CameraContract& camera);

}  /* namespace azp::native_camera */
