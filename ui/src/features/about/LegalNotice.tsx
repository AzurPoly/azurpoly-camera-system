/**
 * Shows the short experimental warning on the main page and the complete legal information on a separate,
 * scrollable page. Its licence texts are the same files that build.ps1 places beside the executable.
 */

import projectLicence from "../../../../LICENCE.md?raw";
import thirdPartyNotices from "../../../../THIRD_PARTY_NOTICES.txt?raw";

type LegalSummaryProps = { appVersion: string; onOpen: () => void };

export function LegalSummary({ appVersion, onOpen }: LegalSummaryProps) {
  return (
    <aside className="legal-summary">
      {appVersion && <p className="legal-summary__version">Version {appVersion}</p>}
      <p>
        This application is experimental, use at your own risk. It is not compatible with Frame Generation or ReShade.
        AzurPoly cannot guarantee that it will operate correctly for every user or system configuration. Please
        contact us if you encounter a problem.
      </p>
      <button type="button" className="legal-summary__link" onClick={onOpen}>
        Legal notice <span aria-hidden="true">›</span>
      </button>
    </aside>
  );
}

type LegalNoticeProps = {
  appVersion: string;
  error: string;
  onBack: () => void;
};

export function LegalNotice({ appVersion, error, onBack }: LegalNoticeProps) {
  return (
    <main className="legal-page">
      <header className="legal-page__header">
        <div>
          <p className="legal-page__eyebrow">Information</p>
          <h1>Legal notice</h1>
        </div>
        <button type="button" className="btn" onClick={onBack}>Back</button>
      </header>

      {error && <p className="error">{error}</p>}

      <div className="legal-page__content">
        <section className="legal-panel">
          <p className="legal-page__eyebrow">Publisher</p>
          <dl className="legal-facts">
            <div><dt>Publisher</dt><dd>AzurPoly</dd></div>
            <div><dt>Version</dt><dd>{appVersion || "—"}</dd></div>
            <div><dt>Copyright</dt><dd>Copyright © 2026 AzurPoly.</dd></div>
          </dl>
        </section>

        <section className="legal-panel">
          <p className="legal-page__eyebrow">Trademarks</p>
          <p>Rafale is a registered trademark of Dassault Aviation.</p>
          <p>All other product names and trademarks are the property of their respective owners.</p>
        </section>

        <section className="legal-panel">
          <p className="legal-page__eyebrow">Open-source licence</p>
          <h2>GPL-3.0-only</h2>
          <p>
            AzurPoly Camera System contains code in common with Taxi Cam. Both projects are licensed under the
            GNU General Public License, version 3 only. You may redistribute and modify this application under
            those terms; it comes without warranty. Taxi Cam is copyright © 2026 Robert Thomson. The complete
            corresponding source code for this application is available from its public GitHub repository.
          </p>
          <details className="legal-licenses">
            <summary>View project licence</summary>
            <pre>{projectLicence}</pre>
          </details>
        </section>

        <section className="legal-panel">
          <p className="legal-page__eyebrow">Third-party software</p>
          <h2>Open-source licenses</h2>
          <p>
            This application includes third-party software distributed under the licenses reproduced below.
          </p>
          <details className="legal-licenses">
            <summary>View third-party notices</summary>
            <pre>{thirdPartyNotices}</pre>
          </details>
        </section>
      </div>
    </main>
  );
}
