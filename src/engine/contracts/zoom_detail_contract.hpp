/*
 * Declares the zoom-detail contract resolver. A successful result locates the model manager's per-piece LOD
 * selection and the one call that runs it for the primary viewpoints, which camera/zoom_detail.cpp redirects,
 * and proves the view field that scales the renderer's mesh fade distances, which zoom_detail.cpp sets for the
 * AZP view. Resolution itself installs nothing and calls no engine function.
 */

#pragma once
#include <cstdint>
#include <string>
#include "code_contract.hpp"

namespace azp::native_camera {

/*
 * Verified locations, resolved once by engine_api.cpp. Code locations are RVAs in the main image; `error`
 * explains why zoom detail is unavailable.
 */
struct ZoomDetailContract {
  bool valid = false;
  std::string error = "Zoom detail unavailable: not resolved.";
  /*
   * int(piece, sphere, viewpoint list, float* size, bool, bool): the LOD of one model piece from one viewpoint
   * list. The E8 call that runs it for the primary list, and the executable section that holds that call
   * (required by the redirect installer's proof).
   */
  std::uint32_t lod_select = 0, primary_call = 0;
  std::uint32_t call_section_rva = 0, call_section_size = 0;
  /*
   * Offset in the view of the mesh fade factor (float): each frame the renderer copies it, divided by a
   * smoothed divisor, to the view's render context and multiplies every mesh's camera distance by it before
   * comparing it with the mesh's far-fade distance.
   */
  std::uint32_t view_fade_scale = 0;
};
ZoomDetailContract resolve_zoom_detail_contract(discovery::ImageReader& reader, const discovery::Inventory& image, std::uintptr_t base);

}  /* namespace azp::native_camera */
