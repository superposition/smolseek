/**
 * BidPanel — Submit private bids for selected cache.
 */

import { useState } from "react";
import type { Cache, GameState } from "../types";

interface BidPanelProps {
  gameState: GameState | null;
  selectedCache: Cache | null;
  onSubmitBid: (cacheId: string, amount: string) => void;
  disabled: boolean;
}

export default function BidPanel({
  gameState,
  selectedCache,
  onSubmitBid,
  disabled,
}: BidPanelProps) {
  const [amount, setAmount] = useState("0.1");
  const [isPending, setIsPending] = useState(false);
  const [lastResult, setLastResult] = useState<string | null>(null);

  const phase = gameState?.phase ?? "LOBBY";
  const isPlayPhase = phase === "PLAY";
  const canBid = isPlayPhase && selectedCache && !selectedCache.collected && !disabled && !isPending;

  const handleBid = async () => {
    if (!selectedCache || !canBid) return;
    setIsPending(true);
    setLastResult(null);

    try {
      // In production: useSend().execute() → get relayId → send to server
      // For now, simulate with a fake relay ID
      const fakeRelayId = `0x${Date.now().toString(16)}`;
      onSubmitBid(selectedCache.id, amount);
      setLastResult(`Bid sent: ${amount} MON → ${selectedCache.id} (relay: ${fakeRelayId.slice(0, 10)}...)`);
    } catch {
      setLastResult("Bid failed");
    } finally {
      setIsPending(false);
    }
  };

  const weiValue = selectedCache?.value
    ? (parseInt(selectedCache.value) / 1e18).toFixed(2)
    : "0";

  return (
    <div className="panel bid-panel">
      <h3>Place Bid</h3>

      {!isPlayPhase && (
        <div className="bid-status muted">
          {phase === "LOBBY" && "Waiting for game to start..."}
          {phase === "DEPOSIT" && "Deposit phase — fund your wallet"}
          {phase === "END" && "Game over!"}
          {phase === "SETUP" && "Setting up..."}
        </div>
      )}

      {isPlayPhase && !selectedCache && (
        <div className="bid-status muted">Click a gold cache on the map to select it</div>
      )}

      {isPlayPhase && selectedCache && (
        <div className="bid-form">
          <div className="selected-cache">
            <span className="cache-name">{selectedCache.id}</span>
            <span className="cache-value">{weiValue} MON</span>
          </div>

          <div className="amount-input">
            <label>Bid Amount (MON)</label>
            <input
              type="number"
              min="0.01"
              step="0.01"
              value={amount}
              onChange={(e) => setAmount(e.target.value)}
              className="input"
              disabled={!canBid}
            />
          </div>

          <button
            onClick={handleBid}
            disabled={!canBid}
            className={`btn btn-bid ${isPending ? "pending" : ""}`}
          >
            {isPending ? "Sending..." : "Submit Private Bid"}
          </button>
        </div>
      )}

      {lastResult && <div className="bid-result">{lastResult}</div>}
    </div>
  );
}
