/**
 * SpectatorView — Full-screen 3D view for projector/judges.
 * No wallet, no bid panel. Just the map + round info overlay.
 */

import type { Cache, GameState, RoundResult } from "../types";
import type { PointCloudData } from "../hooks/useGameSocket";
import MapViewer from "./MapViewer";
import RoundInfo from "./RoundInfo";

interface SpectatorViewProps {
  gameState: GameState | null;
  caches: Cache[];
  roundResult: RoundResult | null;
  navStatus: { status: string; distance_remaining: number } | null;
  robotTarget: { x: number; y: number } | null;
  onPointCloud: (cb: (data: PointCloudData) => void) => void;
  onTrajectory: (cb: (data: Float32Array) => void) => void;
}

export default function SpectatorView({
  gameState,
  caches,
  roundResult,
  navStatus,
  robotTarget,
  onPointCloud,
  onTrajectory,
}: SpectatorViewProps) {
  return (
    <div className="spectator-view">
      <div className="spectator-map">
        <MapViewer
          onPointCloud={onPointCloud}
          onTrajectory={onTrajectory}
          caches={caches}
          selectedCacheId={null}
          onCacheSelect={() => {}}
          robotTarget={robotTarget}
        />
      </div>
      <div className="spectator-overlay">
        <RoundInfo
          gameState={gameState}
          roundResult={roundResult}
          navStatus={navStatus}
        />
        {roundResult?.winner_id && (
          <div className="spectator-result">
            {roundResult.winner_id} collected {roundResult.target_cache_id}!
          </div>
        )}
      </div>
    </div>
  );
}
