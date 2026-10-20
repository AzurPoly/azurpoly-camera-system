import type { CameraSettings, CameraState } from "../domain/camera";
import { createInitialCameraState, sameSettings } from "../domain/camera";
import type { CameraGateway } from "../infrastructure/WebViewCameraGateway";

type StoreListener = (state: CameraState) => void;

/** Settings are sent once a control has rested this long, so dragging a slider sends one change. */
const settleMilliseconds = 250;
/** A change the EXE has not confirmed after this long gives way to the EXE's values. */
const confirmationMilliseconds = 2000;

export class CameraStore {
  private readonly listeners = new Set<StoreListener>();
  private state: CameraState = createInitialCameraState();
  private pending?: CameraSettings;
  private pendingTimer?: number;
  private sentAt = 0;
  private disconnectGateway?: () => void;

  constructor(private readonly gateway: CameraGateway) {}

  start(): void {
    if (this.disconnectGateway) return;

    this.disconnectGateway = this.gateway.subscribe((state) => {
      this.state = state;
      if (this.pending && this.pendingTimer === undefined
        && (sameSettings(state.settings, this.pending) || Date.now() - this.sentAt > confirmationMilliseconds)) {
        this.pending = undefined;
      }
      this.notify();
    });

    this.gateway.send({ type: "refresh" });
  }

  /** The EXE's state, with the settings being edited shown in place of the saved ones. */
  snapshot(): CameraState {
    return this.pending ? { ...this.state, settings: this.pending } : this.state;
  }

  subscribe(listener: StoreListener): () => void {
    this.listeners.add(listener);
    return () => this.listeners.delete(listener);
  }

  changeSettings(settings: CameraSettings): void {
    this.pending = settings;
    window.clearTimeout(this.pendingTimer);
    this.pendingTimer = window.setTimeout(() => {
      this.pendingTimer = undefined;
      this.sentAt = Date.now();
      if (this.pending) this.gateway.send({ type: "setSettings", settings: this.pending });
    }, settleMilliseconds);
    this.notify();
  }

  resetSettings(): void {
    this.changeSettings(this.state.defaults);
  }

  setEnabled(enabled: boolean): void {
    this.gateway.send({ type: "setEnabled", enabled });
  }

  setStartup(enabled: boolean): void {
    this.gateway.send({ type: "setStartup", enabled });
  }

  openLogFolder(): void {
    this.gateway.send({ type: "openLogFolder" });
  }

  quit(): void {
    this.gateway.send({ type: "quit" });
  }

  private notify(): void {
    const state = this.snapshot();
    this.listeners.forEach((listener) => listener(state));
  }
}
