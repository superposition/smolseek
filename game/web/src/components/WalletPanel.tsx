/**
 * WalletPanel — Connect any injected EVM wallet (MetaMask, etc.),
 * show address + MON balance on Monad testnet, then join the game.
 */

import { useState } from "react";
import { useAccount, useBalance, useConnect, useDisconnect } from "wagmi";
import { monadTestnet } from "../wagmi";

interface WalletPanelProps {
  playerId: string | null;
  onJoin: (playerId: string, address: string) => void;
}

export default function WalletPanel({ playerId, onJoin }: WalletPanelProps) {
  const { address, isConnected } = useAccount();
  const { connect, connectors } = useConnect();
  const { disconnect } = useDisconnect();
  const { data: bal } = useBalance({ address, chainId: monadTestnet.id });

  const [name, setName] = useState("");

  const handleJoin = () => {
    if (name.trim() && address) {
      onJoin(name.trim(), address);
    }
  };

  const fmtAddr = address
    ? `${address.slice(0, 6)}...${address.slice(-4)}`
    : "";

  const fmtBal = bal
    ? `${parseFloat(bal.formatted).toFixed(4)} ${bal.symbol}`
    : "—";

  // Connected + joined
  if (playerId && isConnected) {
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
            <span className="value mono">{fmtAddr}</span>
          </div>
          <div className="wallet-row">
            <span className="label">Balance</span>
            <span className="value balance">{fmtBal}</span>
          </div>
        </div>
        <div className="connection-dot connected" />
      </div>
    );
  }

  // Wallet connected but not yet joined
  if (isConnected) {
    return (
      <div className="panel wallet-panel">
        <h3>Join Game</h3>
        <div className="wallet-info" style={{ marginBottom: 8 }}>
          <div className="wallet-row">
            <span className="label">Address</span>
            <span className="value mono">{fmtAddr}</span>
          </div>
          <div className="wallet-row">
            <span className="label">Balance</span>
            <span className="value balance">{fmtBal}</span>
          </div>
        </div>
        <input
          type="text"
          placeholder="Enter your name..."
          value={name}
          onChange={(e) => setName(e.target.value)}
          onKeyDown={(e) => e.key === "Enter" && handleJoin()}
          className="input"
        />
        <div style={{ display: "flex", gap: 8 }}>
          <button onClick={handleJoin} disabled={!name.trim()} className="btn btn-primary" style={{ flex: 1 }}>
            Join
          </button>
          <button onClick={() => disconnect()} className="btn" style={{ opacity: 0.5, fontSize: "0.7rem" }}>
            Disconnect
          </button>
        </div>
      </div>
    );
  }

  // Not connected — show connect button
  return (
    <div className="panel wallet-panel">
      <h3>Connect Wallet</h3>
      {connectors.map((connector) => (
        <button
          key={connector.uid}
          onClick={() => connect({ connector, chainId: monadTestnet.id })}
          className="btn btn-primary"
        >
          {connector.name}
        </button>
      ))}
    </div>
  );
}
