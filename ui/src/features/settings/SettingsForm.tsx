import { useId, type CSSProperties } from "react";
import type { CameraStore } from "../../application/CameraStore";
import { Toggle } from "../../components/Toggle";
import type { CameraSettings, CameraState } from "../../domain/camera";

// Names of the EXE's resolutions, lowest first, and slider steps.
const resolutionNames = ["Low", "Medium", "High"];
const fpsStep = 5;
const exposureStep = 0.1;

/** An exposure shift, signed: "+0.5 EV". */
const signedEv = (ev: number) => `${ev >= 0 ? "+" : ""}${ev.toFixed(1)} EV`;

type SettingsFormProps = { state: CameraState; store: CameraStore };

export function SettingsForm({ state, store }: SettingsFormProps) {
  const { settings, limits, startup } = state;
  const change = (update: Partial<CameraSettings>) => store.changeSettings({ ...settings, ...update });
  // The frame-rate slider's last position means "no limit".
  const unlimited = limits.fpsMax + fpsStep;
  // A resolution saved by an older version sits on the nearest of the three positions until changed.
  const resolutions = limits.resolutions;
  const resolution = resolutions.reduce(
    (best, side, index) => (Math.abs(side - settings.resolution) < Math.abs(resolutions[best] - settings.resolution) ? index : best),
    0,
  );

  return (
    <section className="settings">
      <Slider
        label="Resolution"
        value={resolutions[resolution] === settings.resolution ? resolutionNames[resolution] : `${settings.resolution} px`}
        min={0}
        max={resolutions.length - 1}
        step={1}
        position={resolution}
        disabled={state.resolutionLocked}
        title={state.resolutionLocked ? "Close MSFS to change the resolution." : undefined}
        onChange={(position) => change({ resolution: resolutions[position] })}
      />
      <Slider
        label="Frame rate"
        value={settings.fpsLimit ? `${settings.fpsLimit} fps` : "Unlimited"}
        min={limits.fpsMin}
        max={unlimited}
        step={fpsStep}
        position={settings.fpsLimit || unlimited}
        onChange={(position) => change({ fpsLimit: position > limits.fpsMax ? 0 : position })}
      />
      <Slider
        label="TV exposure"
        value={signedEv(settings.tvExposure)}
        min={limits.exposureMin}
        max={limits.exposureMax}
        step={exposureStep}
        position={settings.tvExposure}
        onChange={(tvExposure) => change({ tvExposure })}
      />
      <Slider
        label="IR exposure"
        value={signedEv(settings.irExposure)}
        min={limits.exposureMin}
        max={limits.exposureMax}
        step={exposureStep}
        position={settings.irExposure}
        onChange={(irExposure) => change({ irExposure })}
      />
      <Switch
        label="Start with MSFS"
        checked={startup.enabled}
        disabled={!startup.available}
        onChange={(enabled) => store.setStartup(enabled)}
      />
      <button type="button" className="settings__reset" onClick={() => store.resetSettings()}>
        RESET DEFAULT
      </button>
    </section>
  );
}

type SliderProps = {
  label: string;
  value: string;
  min: number;
  max: number;
  step: number;
  position: number;
  disabled?: boolean;
  title?: string;
  onChange: (position: number) => void;
};

function Slider({ label, value, min, max, step, position, disabled, title, onChange }: SliderProps) {
  const id = useId();
  return (
    <>
      <label htmlFor={id} title={title}>{label}</label>
      <input
        id={id}
        type="range"
        min={min}
        max={max}
        step={step}
        value={position}
        disabled={disabled}
        title={title}
        style={{ "--fill": (position - min) / (max - min) } as CSSProperties}
        onChange={(event) => onChange(Number(event.target.value))}
      />
      <output htmlFor={id}>{value}</output>
    </>
  );
}

type SwitchProps = { label: string; checked: boolean; disabled?: boolean; onChange: (checked: boolean) => void };

function Switch({ label, checked, disabled, onChange }: SwitchProps) {
  const id = useId();
  return (
    <>
      <label htmlFor={id} className="settings__switch-label">{label}</label>
      <Toggle id={id} checked={checked} disabled={disabled} onChange={onChange} />
    </>
  );
}
