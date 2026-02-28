import { useCallback, useState } from "react";
import { useGameSocket } from "./hooks/useGameSocket";
import MapViewer from "./components/MapViewer";
import WalletPanel from "./components/WalletPanel";
import BidPanel from "./components/BidPanel";
import RoundInfo from "./components/RoundInfo";
import Leaderboard from "./components/Leaderboard";
import SpectatorView from "./components/SpectatorView";
import "./App.css";

function App() {
  const {
    connected,
    gameState,
    caches,
    roundResult,
    robotTarget,
    navStatus,
    sendMessage,
    onPointCloud,
    onTrajectory,
  } = useGameSocket();

  const [playerId, setPlayerId] = useState<string | null>(null);
  const [selectedCacheId, setSelectedCacheId] = useState<string | null>(null);

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
      const fakeRelayId = `0x${Date.now().toString(16)}`;
      sendMessage({
        type: "bid",
        cacheId,
        relayId: fakeRelayId,
        amount: (parseFloat(amount) * 1e18).toString(),
      });
    },
    [sendMessage]
  );

  const handleCacheSelect = useCallback((cacheId: string) => {
    setSelectedCacheId(cacheId);
  }, []);

  const selectedCache = caches.find((c) => c.id === selectedCacheId) ?? null;

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
        <div className={`conn-badge ${connected ? "connected" : "disconnected"}`}>
          {connected ? "Connected" : "Reconnecting..."}
        </div>
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

          <BidPanel
            gameState={gameState}
            selectedCache={selectedCache}
            onSubmitBid={handleBid}
            disabled={!playerId}
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
