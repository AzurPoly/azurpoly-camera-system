import type { CameraCommand, CameraState } from "../domain/camera";

export type CameraStateListener = (state: CameraState) => void;

export interface CameraGateway {
  send(command: CameraCommand): void;
  subscribe(listener: CameraStateListener): () => void;
}

/**
 * Bridge between WebView2 and AZP-Camera.exe
 */
export class WebViewCameraGateway implements CameraGateway {
  private readonly webview = window.chrome?.webview;

  send(command: CameraCommand): void {
    this.webview?.postMessage(command);
  }

  subscribe(listener: CameraStateListener): () => void {
    if (!this.webview) {
      return () => undefined;
    }

    const handleMessage = (event: MessageEvent): void => {
      listener(event.data as CameraState);
    };

    this.webview.addEventListener("message", handleMessage);
    return () => this.webview?.removeEventListener("message", handleMessage);
  }
}
