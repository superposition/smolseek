/**
 * Leaderboard — Player scores sorted by rank.
 */

import type { LeaderboardEntry } from "../types";

interface LeaderboardProps {
  entries: LeaderboardEntry[];
  currentPlayerId: string | null;
}

export default function Leaderboard({ entries, currentPlayerId }: LeaderboardProps) {
  if (entries.length === 0) {
    return (
      <div className="panel leaderboard">
        <h3>Leaderboard</h3>
        <div className="muted">No players yet</div>
      </div>
    );
  }

  return (
    <div className="panel leaderboard">
      <h3>Leaderboard</h3>
      <div className="lb-list">
        {entries.map((entry) => {
          const isMe = entry.id === currentPlayerId;
          const scoreDisplay = entry.score > 0
            ? (entry.score / 1e18).toFixed(2) + " MON"
            : "0 MON";

          return (
            <div
              key={entry.id}
              className={`lb-row ${isMe ? "lb-me" : ""}`}
            >
              <span className="lb-rank">#{entry.rank}</span>
              <span className="lb-name">
                {entry.id}
                {isMe && <span className="lb-you"> (you)</span>}
              </span>
              <span className="lb-score">{scoreDisplay}</span>
            </div>
          );
        })}
      </div>
    </div>
  );
}
