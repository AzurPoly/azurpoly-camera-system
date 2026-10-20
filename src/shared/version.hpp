/*
 * Exposes the build version to native code. build.ps1 generates the included header from version.json so the
 * EXE and DLL report the same version.
 */

#pragma once
#include "azp-camera-version.hpp"

/*
 * Generated from version.json by build.ps1.
 */

#define AZP_CAMERA_WIDEN_IMPL(value) L##value
#define AZP_CAMERA_WIDEN(value) AZP_CAMERA_WIDEN_IMPL(value)
#define AZP_CAMERA_VERSION_WIDE AZP_CAMERA_WIDEN(AZP_CAMERA_VERSION)
