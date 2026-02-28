import { useState } from "react";
import type { Meta, StoryObj } from "@storybook/react";
import type { GameState, Cache, LeaderboardEntry, RoundResult } from "../types";
import type { Transaction } from "../components/TransactionStatus";
import WalletPanel from "../components/WalletPanel";
import BidPanel from "../components/BidPanel";
import RoundInfo from "../components/RoundInfo";
import Leaderboard from "../components/Leaderboard";
import TransactionStatus from "../components/TransactionStatus";
import EscrowPool from "../components/EscrowPool";
import ConnectionBadge from "../components/ConnectionBadge";
import MockMapViewer from "../components/MockMapViewer";
import CameraFeed from "../components/CameraFeed";
import type { ConnState } from "../components/ConnectionBadge";
import "../App.css";

/* ------------------------------------------------------------------ */
/*  Mock data                                                         */
/* ------------------------------------------------------------------ */

const now = Date.now();

const caches: Cache[] = [
  { id: "cache_1", x: 1.2, y: -0.8, z: 0, value: "500000000000000000", collected: true, collected_by: "alice" },
  { id: "cache_2", x: -2.1, y: 3.0, z: 0, value: "1000000000000000000", collected: false, collected_by: null },
  { id: "cache_3", x: 0.5, y: 1.5, z: 0, value: "2000000000000000000", collected: false, collected_by: null },
  { id: "cache_4", x: -1.0, y: -2.5, z: 0, value: "750000000000000000", collected: false, collected_by: null },
  { id: "cache_5", x: 3.3, y: 0.2, z: 0, value: "1500000000000000000", collected: false, collected_by: null },
];

const leaderboard: LeaderboardEntry[] = [
  { rank: 1, id: "alice", score: 2500000000000000000 },
  { rank: 2, id: "you", score: 1000000000000000000 },
  { rank: 3, id: "bob", score: 500000000000000000 },
  { rank: 4, id: "charlie", score: 0 },
];

const playState: GameState = {
  phase: "PLAY",
  current_round: 3,
  total_rounds: 5,
  players: {
    alice: { id: "alice", address: "0xaaa1", score: 2500000000000000000, connected: true },
    you: { id: "you", address: "0xbbb2", score: 1000000000000000000, connected: true },
    bob: { id: "bob", address: "0xccc3", score: 500000000000000000, connected: true },
    charlie: { id: "charlie", address: "0xddd4", score: 0, connected: false },
  },
  caches: Object.fromEntries(caches.map((c) => [c.id, c])),
  leaderboard,
  phase_deadline: now / 1000 + 22,
};

const lobbyState: GameState = {
  ...playState,
  phase: "LOBBY",
  current_round: 0,
  leaderboard: [],
  phase_deadline: null,
};

const endState: GameState = {
  ...playState,
  phase: "END",
  current_round: 5,
  phase_deadline: null,
};

const roundResult: RoundResult = {
  number: 3,
  bids: [
    { player_id: "alice", cache_id: "cache_3", amount: "500000000000000000", relay_id: "abc123", confirmed: true },
    { player_id: "you", cache_id: "cache_3", amount: "200000000000000000", relay_id: "def456", confirmed: true },
  ],
  winner_id: "alice",
  target_cache_id: "cache_3",
  nav_status: null,
};

const transactions: Transaction[] = [
  { relayId: "d4e5f6a7-b8c9-0123-defa-234567890123", phase: "confirmed", label: "Bid on cache_1 (round 1)", timestamp: now - 120000 },
  { relayId: "c3d4e5f6-a7b8-9012-cdef-123456789012", phase: "confirmed", label: "Bid on cache_2 (round 2)", timestamp: now - 60000 },
  { relayId: "b2c3d4e5-f6a7-8901-bcde-f12345678901", phase: "confirming", label: "Distribute to alice", timestamp: now - 8000 },
  { relayId: "a1b2c3d4-e5f6-7890-abcd-ef1234567890", phase: "proving", label: "Bid on cache_3 (round 3)", timestamp: now - 2000 },
];

/* ------------------------------------------------------------------ */
/*  Composed layout                                                   */
/* ------------------------------------------------------------------ */

function MockGameLayout({
  gameState,
  selectedCacheInit,
  txs,
  result,
  navStatus,
  wsState = "ok",
  escrowState = "ok",
}: {
  gameState: GameState;
  selectedCacheInit: Cache | null;
  txs: Transaction[];
  result: RoundResult | null;
  navStatus: { status: string; distance_remaining: number } | null;
  wsState?: ConnState;
  escrowState?: ConnState;
}) {
  const [selectedCacheId, setSelectedCacheId] = useState<string | null>(
    selectedCacheInit?.id ?? null
  );
  const selectedCache = caches.find((c) => c.id === selectedCacheId) ?? null;

  return (
    <div className="app" style={{ height: "100vh", minHeight: 700 }}>
      <header className="app-header">
        <h1>smolseek</h1>
        <ConnectionBadge websocket={wsState} escrow={escrowState} />
      </header>

      <div className="app-body">
        <div className="map-container" style={{ position: "relative" }}>
          <MockMapViewer
            caches={caches}
            selectedCacheId={selectedCacheId}
            onCacheSelect={setSelectedCacheId}
          />
          {/* Camera feed top-right */}
          <div style={{
            position: "absolute",
            top: 12,
            right: 12,
            zIndex: 2,
          }}>
            <CameraFeed rosbridgeUrl="ws://192.168.0.221:9090" />
          </div>
          {/* Escrow overlay bottom-left */}
          <div style={{
            position: "absolute",
            bottom: 40,
            left: 12,
            zIndex: 1,
            pointerEvents: "none",
            opacity: 0.9,
          }}>
            <EscrowPool
              address="unlink1qyz0qu97u8x6mwtag7tylwtkuhxqyyvchsvndaxggv27pnu6du4gpz53jwtfr9y2jy"
              balance="3500000000000000000"
            />
          </div>
        </div>

        <div className="side-panel">
          <RoundInfo gameState={gameState} roundResult={result} navStatus={navStatus} />
          <WalletPanel playerId="you" onJoin={() => {}} />
          <BidPanel
            gameState={gameState}
            selectedCache={selectedCache}
            onSubmitBid={() => {}}
            disabled={escrowState === "err"}
          />
          <TransactionStatus transactions={txs} />
          <Leaderboard entries={gameState.leaderboard} currentPlayerId="you" />
        </div>
      </div>
    </div>
  );
}

/* ------------------------------------------------------------------ */
/*  Stories                                                           */
/* ------------------------------------------------------------------ */

const meta: Meta = {
  title: "Full App/MockGame",
  parameters: { layout: "fullscreen" },
};

export default meta;
type Story = StoryObj;

export const MidGame: Story = {
  render: () => (
    <MockGameLayout
      gameState={playState}
      selectedCacheInit={caches[2]}
      txs={transactions}
      result={null}
      navStatus={null}
    />
  ),
};

export const RobotNavigating: Story = {
  render: () => (
    <MockGameLayout
      gameState={playState}
      selectedCacheInit={caches[2]}
      txs={transactions.slice(0, 3)}
      result={null}
      navStatus={{ status: "navigating", distance_remaining: 1.4 }}
    />
  ),
};

export const RoundWon: Story = {
  render: () => (
    <MockGameLayout
      gameState={playState}
      selectedCacheInit={null}
      txs={transactions}
      result={roundResult}
      navStatus={null}
    />
  ),
};

export const Lobby: Story = {
  render: () => (
    <MockGameLayout
      gameState={lobbyState}
      selectedCacheInit={null}
      txs={[]}
      result={null}
      navStatus={null}
    />
  ),
};

export const GameOver: Story = {
  render: () => (
    <MockGameLayout
      gameState={endState}
      selectedCacheInit={null}
      txs={transactions.map((t) => ({ ...t, phase: "confirmed" as const }))}
      result={roundResult}
      navStatus={null}
    />
  ),
};

export const FailedTransaction: Story = {
  render: () => (
    <MockGameLayout
      gameState={playState}
      selectedCacheInit={caches[3]}
      txs={[
        ...transactions.slice(0, 2),
        {
          relayId: "e5f6a7b8-c9d0-1234-efab-345678901234",
          phase: "failed" as const,
          label: "Bid on cache_4 (round 2)",
          timestamp: now - 40000,
          error: "Insufficient escrow balance",
        },
        transactions[3],
      ]}
      result={null}
      navStatus={null}
    />
  ),
};

export const EscrowDown: Story = {
  render: () => (
    <MockGameLayout
      gameState={playState}
      selectedCacheInit={caches[2]}
      txs={transactions.slice(0, 2)}
      result={null}
      navStatus={null}
      escrowState="err"
    />
  ),
};
