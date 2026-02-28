/**
 * useGameSocket — WebSocket hook connecting to game server (:8081).
 *
 * Parses binary messages (map data 0x01-0x04, game data 0x10-0x14).
 * Provides sendMessage() for text commands (join, bid, start).
 * Auto-reconnects on disconnect.
 */

import { useCallback, useEffect, useRef, useState } from "react";
import type { Cache, GameState, RoundResult } from "../types";
import { MSG } from "../types";

const GAME_WS_PORT = 8081;
const RECONNECT_MS = 3000;

export interface PointCloudData {
  points: Float32Array;
  isDelta: boolean;
}

export interface RobotTarget {
  cache_id: string;
  x: number;
  y: number;
}

export interface NavStatus {
  status: string;
  cache_id: string;
  distance_remaining: number;
}

interface GameSocketState {
  connected: boolean;
  gameState: GameState | null;
  caches: Cache[];
  roundResult: RoundResult | null;
  robotTarget: RobotTarget | null;
  navStatus: NavStatus | null;
}

export function useGameSocket() {
  const wsRef = useRef<WebSocket | null>(null);
  const [state, setState] = useState<GameSocketState>({
    connected: false,
    gameState: null,
    caches: [],
    roundResult: null,
    robotTarget: null,
    navStatus: null,
  });

  // Callbacks for components that need raw binary data (MapViewer)
  const pointCloudCb = useRef<((data: PointCloudData) => void) | null>(null);
  const trajectoryCb = useRef<((data: Float32Array) => void) | null>(null);

  const onPointCloud = useCallback((cb: (data: PointCloudData) => void) => {
    pointCloudCb.current = cb;
  }, []);

  const onTrajectory = useCallback((cb: (data: Float32Array) => void) => {
    trajectoryCb.current = cb;
  }, []);

  const sendMessage = useCallback((data: Record<string, unknown>) => {
    if (wsRef.current?.readyState === WebSocket.OPEN) {
      wsRef.current.send(JSON.stringify(data));
    }
  }, []);

  useEffect(() => {
    let reconnectTimer: ReturnType<typeof setTimeout>;
    let ws: WebSocket;

    function connect() {
      const host = window.location.hostname || "localhost";
      const url = `ws://${host}:${GAME_WS_PORT}`;
      ws = new WebSocket(url);
      ws.binaryType = "arraybuffer";
      wsRef.current = ws;

      ws.onopen = () => {
        setState((s) => ({ ...s, connected: true }));
      };

      ws.onclose = () => {
        setState((s) => ({ ...s, connected: false }));
        wsRef.current = null;
        reconnectTimer = setTimeout(connect, RECONNECT_MS);
      };

      ws.onerror = () => ws.close();

      ws.onmessage = (event) => {
        if (typeof event.data === "string") {
          // Text messages (join confirmation, bid results)
          try {
            const data = JSON.parse(event.data);
            // Could dispatch to a text message handler if needed
            console.log("WS text:", data);
          } catch {
            // ignore
          }
          return;
        }

        const buf = event.data as ArrayBuffer;
        if (buf.byteLength < 1) return;

        const view = new DataView(buf);
        const msgType = view.getUint8(0);
        const payload = buf.slice(1);

        switch (msgType) {
          case MSG.FULL_CLOUD:
            pointCloudCb.current?.({
              points: new Float32Array(payload),
              isDelta: false,
            });
            break;

          case MSG.DELTA_POINTS:
            pointCloudCb.current?.({
              points: new Float32Array(payload),
              isDelta: true,
            });
            break;

          case MSG.TRAJECTORY:
            trajectoryCb.current?.(new Float32Array(payload));
            break;

          case MSG.STATUS:
            // Map status (0x04) — could expose if needed
            break;

          case MSG.GAME_STATE: {
            const gs = parseJSON<GameState>(payload);
            if (gs) setState((s) => ({ ...s, gameState: gs }));
            break;
          }

          case MSG.CACHE_LOCS: {
            const caches = parseJSON<Cache[]>(payload);
            if (caches) setState((s) => ({ ...s, caches }));
            break;
          }

          case MSG.ROBOT_TARGET: {
            const rt = parseJSON<RobotTarget>(payload);
            if (rt) setState((s) => ({ ...s, robotTarget: rt }));
            break;
          }

          case MSG.ROUND_RESULT: {
            const rr = parseJSON<RoundResult>(payload);
            if (rr) setState((s) => ({ ...s, roundResult: rr }));
            break;
          }

          case MSG.NAV_STATUS: {
            const ns = parseJSON<NavStatus>(payload);
            if (ns) setState((s) => ({ ...s, navStatus: ns }));
            break;
          }
        }
      };
    }

    connect();

    return () => {
      clearTimeout(reconnectTimer);
      ws?.close();
    };
  }, []);

  return {
    ...state,
    sendMessage,
    onPointCloud,
    onTrajectory,
  };
}

function parseJSON<T>(buffer: ArrayBuffer): T | null {
  try {
    const text = new TextDecoder().decode(buffer);
    return JSON.parse(text) as T;
  } catch {
    return null;
  }
}
