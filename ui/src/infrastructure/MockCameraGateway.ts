import { createInitialCameraState } from "../domain/camera";
import type { CameraCommand, CameraState } from "../domain/camera";
import type { CameraGateway, CameraStateListener } from "./WebViewCameraGateway";

/** Browser-only state provider used by the Vite development server. */
export class MockCameraGateway implements CameraGateway {
  private readonly listeners = new Set<CameraStateListener>();
  private timer?: number;
  private state: CameraState = {
    ...createInitialCameraState(),
    appVersion: "UI preview",
    status: { text: "Streaming", streaming: true },
    startup: { enabled: true, available: true },
    fps: "44.8",
  };

  send(command: CameraCommand): void {
    switch (command.type) {
      case "refresh":
      case "openLogFolder":
      case "quit":
        break;
      case "setEnabled":
        this.state = {
          ...this.state,
          enabled: command.enabled,
          status: command.enabled ? { text: "Streaming", streaming: true } : { text: "Off", streaming: false },
          fps: command.enabled ? "44.8" : "",
        };
        break;
      case "setSettings":
        this.state = { ...this.state, settings: command.settings };
        break;
      case "setStartup":
        this.state = { ...this.state, startup: { ...this.state.startup, enabled: command.enabled } };
        break;
    }
    window.setTimeout(() => this.publish(), 80);
  }

  subscribe(listener: CameraStateListener): () => void {
    this.listeners.add(listener);
    this.timer ??= window.setInterval(() => this.publish(), 250);
    return () => {
      this.listeners.delete(listener);
      if (this.listeners.size === 0 && this.timer !== undefined) {
        window.clearInterval(this.timer);
        this.timer = undefined;
      }
    };
  }

  private publish(): void {
    this.listeners.forEach((listener) => listener(this.state));
  }
}
