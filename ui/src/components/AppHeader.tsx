import logo from "../../../assets/branding/azp-camera-icon.png";
// Written by build.ps1 from assets/background.png at the displayed size.
import picture from "../generated/background.png";
import type { CameraState } from "../domain/camera";
import { StatusDot } from "./StatusDot";

export function AppHeader({ state }: { state: CameraState }) {
  return (
    <header className="header">
      <img className="header__logo" src={logo} alt="AzurPoly" />
      <div>
        <p className="header__brand">AzurPoly</p>
        <h1>Rafale Camera System</h1>
        <p className={`header__status${state.status.streaming ? " header__status--active" : ""}`}>
          <StatusDot active={state.status.streaming} />
          {state.status.text}
        </p>
        {state.fps && <p className="header__fps">{state.fps} FPS</p>}
      </div>
      <img className="header__picture" src={picture} alt="" />
    </header>
  );
}
