/*
 * Writes this DLL load's files into the EXE's log folder: log\<YYYY-MM-DD> next to azp-camera.dll, like the
 * EXE's own log, or %LOCALAPPDATA%\AzurPoly\CameraSystem\log\<YYYY-MM-DD> when that folder cannot be created.
 *
 * - bridge-<HHMMSS>.log: the control loop's status events, capped at 16 MB.
 * - diagnostic-<HHMMSS>.txt: a one-time report on the simulator for remote support. It covers the AZP build,
 *   Windows, the MSFS executable (path, distribution, version, PE identity, sections), every engine contract
 *   with the evidence of templates that were not found, the modules loaded in MSFS, the graphics adapters,
 *   and the simulator's exe.xml and UserCfg.opt with its Community folder.
 *
 * <HHMMSS> is the local time of the first write. Everything here only reads and writes its own files; a
 * failure loses a line or the report, never the camera.
 */

#pragma once
#include <string>
#include "../engine/engine_api.hpp"

namespace azp::diagnostic {

/*
 * Appends one line to this DLL load's bridge log.
 */
void append_bridge_log(const std::string& line);
/*
 * Writes the report once; later calls do nothing. Call from the DLL control thread after the engine contracts
 * are resolved.
 */
void write_report(const native_camera::EngineApi& api);

}  /* namespace azp::diagnostic */
