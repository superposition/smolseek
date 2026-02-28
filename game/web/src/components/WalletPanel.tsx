/**
 * WalletPanel — Wallet connection, balance display, and deposit.
 *
 * For now uses a simplified mock since @unlink-xyz/react requires
 * a real browser wallet. In production, wrap with UnlinkProvider.
 */

import { useState } from "react";

interface WalletPanelProps {
  playerId: string | null;
  onJoin: (playerId: string, address: string) => void;
}

export default function WalletPanel({ playerId, onJoin }: WalletPanelProps) {
  const [name, setName] = useState("");
  const [address] = useState(() => `0x${Math.random().toString(16).slice(2, 10)}`);
  const [balance] = useState("1.5 MON");

  const handleJoin = () => {
    if (name.trim()) {
      onJoin(name.trim(), address);
    }
  };

  if (playerId) {
    return (
      <div className="panel wallet-panel">
        <h3>Wallet</h3>
        <div className="wallet-info">
          <div className="wallet-row">
            <span className="label">Player</span>
            <span className="value">{playerId}</span>
          </div>
          <div className="wallet-row">
            <span className="label">Address</span>
            <span className="value mono">{address.slice(0, 6)}...{address.slice(-4)}</span>
          </div>
          <div className="wallet-row">
            <span className="label">Balance</span>
            <span className="value balance">{balance}</span>
          </div>
        </div>
        <div className="connection-dot connected" />
      </div>
    );
  }

  return (
    <div className="panel wallet-panel">
      <h3>Join Game</h3>
      <input
        type="text"
        placeholder="Enter your name..."
        value={name}
        onChange={(e) => setName(e.target.value)}
        onKeyDown={(e) => e.key === "Enter" && handleJoin()}
        className="input"
      />
      <button onClick={handleJoin} disabled={!name.trim()} className="btn btn-primary">
        Join
      </button>
    </div>
  );
}
