type ToggleProps = {
  id: string;
  checked: boolean;
  disabled?: boolean;
  /** Extra class, e.g. `toggle--power` for the large green/red camera switch. */
  className?: string;
  onChange: (checked: boolean) => void;
};

export function Toggle({ id, checked, disabled, className, onChange }: ToggleProps) {
  return (
    <input
      id={id}
      className={className ? `toggle ${className}` : "toggle"}
      type="checkbox"
      role="switch"
      checked={checked}
      disabled={disabled}
      onChange={(event) => onChange(event.target.checked)}
    />
  );
}
