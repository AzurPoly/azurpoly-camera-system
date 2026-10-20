import type { CameraStore } from "../../application/CameraStore";

export function Actions({ store }: { store: CameraStore }) {
  return (
    <div className="actions">
      <button type="button" className="btn btn--quit" onClick={() => store.quit()}>QUIT</button>
      <button type="button" className="btn" onClick={() => store.openLogFolder()}>OPEN LOGS</button>
    </div>
  );
}
