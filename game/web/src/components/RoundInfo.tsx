/**
 * RoundInfo — Round counter, countdown timer, and phase label.
 */

import { useEffect, useState } from "react";
import type { GameState, RoundResult } from "../types";

interface RoundInfoProps {
  gameState: GameState | null;
  roundResult: RoundResult | null;
  navStatus: { status: string; distance_remaining: number } | null;
}

export default function RoundInfo({ gameState, roundResult, navStatus }: RoundInfoProps) {
  const [countdown, setCountdown] = useState<number | null>(null);

  useEffect(() => {
    if (!gameState?.phase_deadline) {
      setCountdown(null);
      return;
    }

    const tick = () => {
      const remaining = Math.max(0, Math.ceil(gameState.phase_deadline! - Date.now() / 1000));
      setCountdown(remaining);
    };

    tick();
    const timer = setInterval(tick, 500);
    return () => clearInterval(timer);
  }, [gameState?.phase_deadline]);

  const phase = gameState?.phase ?? "LOBBY";
  const round = gameState?.current_round ?? 0;
  const totalRounds = gameState?.total_rounds ?? 0;

  const statusLabel = (() => {
    if (phase === "LOBBY") return "Waiting for players";
    if (phase === "DEPOSIT") return "Deposit phase";
    if (phase === "END") return "Game over";
    if (phase !== "PLAY") return phase;

    if (navStatus?.status === "navigating") {
      return `Robot moving — ${navStatus.distance_remaining.toFixed(1)}m`;
    }
    if (roundResult?.winner_id && round === roundResult.number) {
      return `${roundResult.winner_id} won round ${round}!`;
    }
    return "Bidding open";
  })();

  return (
    <div className="panel round-info">
      <div className="phase-badge">{phase}</div>

      {phase === "PLAY" && (
        <div className="round-counter">
          Round {round} / {totalRounds}
        </div>
      )}

      {countdown !== null && countdown > 0 && (
        <div className="countdown">
          <span className="countdown-num">{countdown}</span>
          <span className="countdown-label">s</span>
        </div>
      )}

      <div className="status-label">{statusLabel}</div>
    </div>
  );
}
