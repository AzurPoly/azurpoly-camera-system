/**
 * The state AZP-Camera.exe publishes four times per second and the commands the interface sends back.
 * The EXE is the authority: every value below is already validated and formatted there.
 */

/** Native camera settings, saved by the EXE and applied to the running camera. */
export type CameraSettings = {
  /** Side of the square image MSFS renders, in pixels. The aircraft always receives it scaled to 720 x 720. */
  resolution: number;
  /** Maximum rendered frames per second; 0 renders every simulator frame. */
  fpsLimit: number;
  /** TV exposure, in EV: a gain on the camera's final TV image. */
  tvExposure: number;
  /** IR exposure, in EV: shifts the brightness the IR automatic exposure aims at. */
  irExposure: number;
};

export type SettingLimits = {
  /** The Low, Medium and High resolutions, in pixels. */
  resolutions: number[];
  fpsMin: number;
  fpsMax: number;
  /** Bounds of both exposure sliders, in EV. */
  exposureMin: number;
  exposureMax: number;
};

export type CameraState = {
  appVersion: string;
  /** The ON/OFF switch: OFF pauses the camera, and the aircraft sees AZP Camera as absent. */
  enabled: boolean;
  /** The header's status word, and whether video is streaming. */
  status: { text: string; streaming: boolean };
  settings: CameraSettings;
  defaults: CameraSettings;
  limits: SettingLimits;
  /** MSFS is running: the resolution can only be changed while it is closed. */
  resolutionLocked: boolean;
  /** `available`: at least one MSFS 2024 installation was found. */
  startup: { enabled: boolean; available: boolean };
  /** Video input rate, in images per second, already formatted; empty while not streaming. */
  fps: string;
  error: string;
};

export type CameraCommand =
  | { type: "refresh" }
  | { type: "setEnabled"; enabled: boolean }
  | { type: "setSettings"; settings: CameraSettings }
  | { type: "setStartup"; enabled: boolean }
  | { type: "openLogFolder" }
  | { type: "quit" };

const defaultSettings: CameraSettings = { resolution: 720, fpsLimit: 0, tvExposure: 0, irExposure: 0 };

export function createInitialCameraState(): CameraState {
  return {
    appVersion: "",
    enabled: true,
    status: { text: "", streaming: false },
    settings: defaultSettings,
    defaults: defaultSettings,
    limits: { resolutions: [480, 720, 1080], fpsMin: 5, fpsMax: 120, exposureMin: -2, exposureMax: 2 },
    resolutionLocked: false,
    startup: { enabled: false, available: false },
    fps: "",
    error: "",
  };
}

export function sameSettings(a: CameraSettings, b: CameraSettings): boolean {
  return a.resolution === b.resolution && a.fpsLimit === b.fpsLimit && a.tvExposure === b.tvExposure
    && a.irExposure === b.irExposure;
}
