/**
 * TransactionStatus — Shows relay ID, ZK proof status, and confirmation progress.
 */

import { useEffect, useState } from "react";

export type TxPhase = "idle" | "proving" | "relaying" | "confirming" | "confirmed" | "failed";

export interface Transaction {
  relayId: string;
  phase: TxPhase;
  label: string;
  timestamp: number;
  error?: string;
}

interface TransactionStatusProps {
  transactions: Transaction[];
  maxVisible?: number;
}

const PHASE_LABEL: Record<TxPhase, string> = {
  idle: "",
  proving: "Generating ZK proof",
  relaying: "Submitting to relay",
  confirming: "Awaiting confirmation",
  confirmed: "Confirmed",
  failed: "Failed",
};

function ElapsedTimer({ since }: { since: number }) {
  const [elapsed, setElapsed] = useState(0);

  useEffect(() => {
    const interval = setInterval(() => setElapsed(Math.floor((Date.now() - since) / 1000)), 500);
    return () => clearInterval(interval);
  }, [since]);

  return <span className="tx-elapsed">{elapsed}s</span>;
}

function TxRow({ tx }: { tx: Transaction }) {
  const isActive = tx.phase !== "confirmed" && tx.phase !== "failed" && tx.phase !== "idle";
  const shortId = tx.relayId.length > 16
    ? `${tx.relayId.slice(0, 8)}...${tx.relayId.slice(-6)}`
    : tx.relayId;

  return (
    <div className={`tx-row tx-${tx.phase}`}>
      <div className="tx-row-top">
        <span className="tx-label">{tx.label}</span>
        {isActive && <ElapsedTimer since={tx.timestamp} />}
      </div>
      <div className="tx-row-bottom">
        <span className="tx-id">{shortId}</span>
        <span className={`tx-phase-badge tx-phase-${tx.phase}`}>
          {isActive && <span className="tx-spinner" />}
          {PHASE_LABEL[tx.phase]}
        </span>
      </div>
      {tx.phase === "failed" && tx.error && (
        <div className="tx-error">{tx.error}</div>
      )}
    </div>
  );
}

export default function TransactionStatus({
  transactions,
  maxVisible = 5,
}: TransactionStatusProps) {
  const visible = transactions.slice(-maxVisible).reverse();

  if (visible.length === 0) return null;

  return (
    <div className="panel tx-status-panel">
      <h3>Transactions</h3>
      <div className="tx-list">
        {visible.map((tx) => (
          <TxRow key={tx.relayId} tx={tx} />
        ))}
      </div>
    </div>
  );
}
