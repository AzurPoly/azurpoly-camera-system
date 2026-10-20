import { useState } from "react";
import type { CameraStore } from "./application/CameraStore";
import { AppHeader } from "./components/AppHeader";
import { Toggle } from "./components/Toggle";
import type { CameraState } from "./domain/camera";
import { LegalNotice, LegalSummary } from "./features/about/LegalNotice";
import { Actions } from "./features/application/Actions";
import { SettingsForm } from "./features/settings/SettingsForm";

type AppProps = {
  camera: CameraState;
  store: CameraStore;
};

export default function App({ camera, store }: AppProps) {
  const [legalOpen, setLegalOpen] = useState(false);

  if (legalOpen) {
    return (
      <LegalNotice
        appVersion={camera.appVersion}
        error={camera.error}
        onBack={() => setLegalOpen(false)}
      />
    );
  }

  return (
    <main className="app">
      <AppHeader state={camera} />
      <SettingsForm state={camera} store={store} />
      {camera.error && <p className="error">{camera.error}</p>}
      <Actions store={store} />
      <div className="power">
        <Toggle
          id="camera-enabled"
          className="toggle--power"
          checked={camera.enabled}
          onChange={(enabled) => store.setEnabled(enabled)}
        />
        <label htmlFor="camera-enabled" className={camera.enabled ? "power--on" : undefined}>
          {camera.enabled ? "Camera enabled" : "Camera disabled"}
        </label>
      </div>
      <LegalSummary appVersion={camera.appVersion} onOpen={() => setLegalOpen(true)} />
    </main>
  );
}
