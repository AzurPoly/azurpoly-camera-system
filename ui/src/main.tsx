import { StrictMode } from "react";
import { createRoot } from "react-dom/client";
import { CameraStore } from "./application/CameraStore";
import App from "./App";
import { MockCameraGateway } from "./infrastructure/MockCameraGateway";
import { WebViewCameraGateway } from "./infrastructure/WebViewCameraGateway";
import "./styles/main.scss";

const gateway = import.meta.env.DEV && !window.chrome?.webview
  ? new MockCameraGateway()
  : new WebViewCameraGateway();
const cameraStore = new CameraStore(gateway);
const root = createRoot(document.getElementById("root")!);

function renderApplication(): void {
  root.render(
    <StrictMode><App camera={cameraStore.snapshot()} store={cameraStore} /></StrictMode>,
  );
}

cameraStore.subscribe(renderApplication);
renderApplication();
cameraStore.start();
