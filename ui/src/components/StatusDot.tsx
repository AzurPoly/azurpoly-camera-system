export function StatusDot({ active }: { active: boolean }) {
  return <span className={`status-dot${active ? " status-dot--active" : ""}`} />;
}
