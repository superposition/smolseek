import { useCallback, useRef, useState } from "react";
import { useGameSocket } from "./hooks/useGameSocket";
import MapViewer from "./components/MapViewer";
import WalletPanel from "./components/WalletPanel";
import BidPanel from "./components/BidPanel";
import RoundInfo from "./components/RoundInfo";
import Leaderboard from "./components/Leaderboard";
import SpectatorView from "./components/SpectatorView";
import CacheCard from "./components/CacheCard";
import ConnectionBadge from "./components/ConnectionBadge";
import TransactionStatus from "./components/TransactionStatus";
import EscrowPool from "./components/EscrowPool";
import type { Transaction, TxPhase } from "./components/TransactionStatus";
import CameraFeed from "./components/CameraFeed";
import type { ConnState } from "./components/ConnectionBadge";
import "./App.css";

function App() {
  const {
    connected,
    gameState,
    caches,
    roundResult,
    robotTarget,
    navStatus,
    mapStatus,
    sendMessage,
    onPointCloud,
    onTrajectory,
  } = useGameSocket();

  const [playerId, setPlayerId] = useState<string | null>(null);
  const [selectedCacheId, setSelectedCacheId] = useState<string | null>(null);
  const [transactions, setTransactions] = useState<Transaction[]>([]);
  const txTimers = useRef<ReturnType<typeof setTimeout>[]>([]);

  // Check for spectator mode
  const isSpectator = window.location.search.includes("mode=spectator")
    || window.location.pathname.includes("/spectator");

  const handleJoin = useCallback(
    (name: string, address: string) => {
      sendMessage({ type: "join", playerId: name, address });
      setPlayerId(name);
    },
    [sendMessage]
  );

  const handleStartGame = useCallback(() => {
    sendMessage({ type: "start" });
  }, [sendMessage]);

  const handleBid = useCallback(
    (cacheId: string, amount: string) => {
      const relayId = `0x${Date.now().toString(16)}`;
      sendMessage({
        type: "bid",
        cacheId,
        relayId,
        amount: (parseFloat(amount) * 1e18).toString(),
      });

      // Track transaction with simulated ZK pipeline phases
      const tx: Transaction = {
        relayId,
        phase: "proving",
        label: `Bid ${amount} MON → ${cacheId}`,
        timestamp: Date.now(),
      };
      setTransactions((prev) => [...prev, tx]);

      const updatePhase = (id: string, phase: TxPhase) =>
        setTransactions((prev) =>
          prev.map((t) => (t.relayId === id ? { ...t, phase } : t))
        );

      // Simulate: proving (1.5s) → relaying (1s) → confirming (2s) → confirmed
      const t1 = setTimeout(() => updatePhase(relayId, "relaying"), 1500);
      const t2 = setTimeout(() => updatePhase(relayId, "confirming"), 2500);
      const t3 = setTimeout(() => updatePhase(relayId, "confirmed"), 4500);
      txTimers.current.push(t1, t2, t3);
    },
    [sendMessage]
  );

  const handleCacheSelect = useCallback((cacheId: string) => {
    setSelectedCacheId(cacheId);
  }, []);

  const selectedCache = caches.find((c) => c.id === selectedCacheId) ?? null;

  // Connection health for header badge
  const wsState: ConnState = connected ? "ok" : "err";
  const escrowState: ConnState = connected && gameState ? "ok" : connected ? "warn" : "err";

  if (isSpectator) {
    return (
      <SpectatorView
        gameState={gameState}
        caches={caches}
        roundResult={roundResult}
        navStatus={navStatus}
        robotTarget={robotTarget}
        onPointCloud={onPointCloud}
        onTrajectory={onTrajectory}
      />
    );
  }

  return (
    <div className="app">
      <header className="app-header">
        <h1>smolseek</h1>
        <ConnectionBadge websocket={wsState} escrow={escrowState} />
      </header>

      <div className="app-body">
        <div className="map-container">
          <MapViewer
            onPointCloud={onPointCloud}
            onTrajectory={onTrajectory}
            caches={caches}
            selectedCacheId={selectedCacheId}
            onCacheSelect={handleCacheSelect}
            robotTarget={robotTarget}
          />
          <div style={{ position: "absolute", top: 12, right: 12, zIndex: 2 }}>
            <CameraFeed />
          </div>
          {mapStatus && (
            <div className="map-overlay">
              <span className="map-stat">{mapStatus.total_points.toLocaleString()} pts</span>
              <span className="map-stat">{mapStatus.tracking_state}</span>
              {mapStatus.total_keyframes > 0 && (
                <span className="map-stat">{mapStatus.total_keyframes} kf</span>
              )}
            </div>
          )}
        </div>

        <div className="side-panel">
          <RoundInfo
            gameState={gameState}
            roundResult={roundResult}
            navStatus={navStatus}
          />

          <WalletPanel playerId={playerId} onJoin={handleJoin} />

          {playerId && gameState?.phase === "LOBBY" && (
            <div className="panel">
              <button onClick={handleStartGame} className="btn btn-start">
                Start Game
              </button>
            </div>
          )}

          {caches.length > 0 && (
            <div className="panel cache-list-panel">
              <h3>Caches</h3>
              <div className="cache-list">
                {caches.map((c) => (
                  <CacheCard
                    key={c.id}
                    cache={c}
                    selected={c.id === selectedCacheId}
                    onClick={handleCacheSelect}
                  />
                ))}
              </div>
            </div>
          )}

          <BidPanel
            gameState={gameState}
            selectedCache={selectedCache}
            onSubmitBid={handleBid}
            disabled={!playerId}
          />

          <TransactionStatus transactions={transactions} />

          <EscrowPool
            address="0x000000000000000000000000000000000000dEaD"
            balance={gameState?.phase === "PLAY" ? "500000000000000000" : "0"}
          />

          <Leaderboard
            entries={gameState?.leaderboard ?? []}
            currentPlayerId={playerId}
          />
        </div>
      </div>
    </div>
  );
}

export default App;
