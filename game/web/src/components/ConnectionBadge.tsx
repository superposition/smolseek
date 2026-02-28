/**
 * ConnectionBadge — WebSocket + escrow service health indicator.
 */

export type ConnState = "ok" | "err" | "warn";

interface ConnectionBadgeProps {
  websocket: ConnState;
  escrow: ConnState;
}

function Dot({ state, label }: { state: ConnState; label: string }) {
  return (
    <div className="conn-indicator">
      <span className={`conn-dot ${state}`} />
      <span className="conn-name">{label}</span>
    </div>
  );
}

export default function ConnectionBadge({ websocket, escrow }: ConnectionBadgeProps) {
  return (
    <div className="conn-status">
      <Dot state={websocket} label="WS" />
      <Dot state={escrow} label="Escrow" />
    </div>
  );
}
