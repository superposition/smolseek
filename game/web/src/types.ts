/** Shared TypeScript types matching game/coordinator/models.py */

export interface Player {
  id: string;
  address: string | null;
  score: number;
  connected: boolean;
}

export interface Cache {
  id: string;
  x: number;
  y: number;
  z: number;
  value: string; // MON wei
  collected: boolean;
  collected_by: string | null;
}

export interface Bid {
  player_id: string;
  cache_id: string;
  amount: string;
  relay_id: string;
  confirmed: boolean;
}

export interface RoundResult {
  number: number;
  bids: Bid[];
  winner_id: string | null;
  target_cache_id: string | null;
  nav_status: string | null;
}

export interface LeaderboardEntry {
  rank: number;
  id: string;
  score: number;
}

export interface GameState {
  phase: "LOBBY" | "SETUP" | "DEPOSIT" | "PLAY" | "END";
  current_round: number;
  total_rounds: number;
  players: Record<string, Player>;
  caches: Record<string, Cache>;
  leaderboard: LeaderboardEntry[];
  phase_deadline: number | null;
}

/** WS binary protocol message types */
export const MSG = {
  FULL_CLOUD:   0x01,
  DELTA_POINTS: 0x02,
  TRAJECTORY:   0x03,
  STATUS:       0x04,
  GAME_STATE:   0x10,
  CACHE_LOCS:   0x11,
  ROBOT_TARGET: 0x12,
  ROUND_RESULT: 0x13,
  NAV_STATUS:   0x14,
} as const;
