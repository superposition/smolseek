/**
 * CameraFeed — small overlay showing the robot's camera stream.
 * Connects to rosbridge (ws://host:9090) and subscribes to a
 * sensor_msgs/CompressedImage topic for low-bandwidth JPEG frames.
 * Falls back to a static MJPEG <img> URL if provided.
 */

import { useEffect, useRef, useState } from "react";

interface CameraFeedProps {
  /** rosbridge WebSocket URL, e.g. ws://192.168.0.221:9090 */
  rosbridgeUrl?: string;
  /** ROS topic for compressed image */
  topic?: string;
  /** Fallback: direct MJPEG stream URL (used if no rosbridge) */
  mjpegUrl?: string;
  /** Throttle: max fps to render (default 10) */
  maxFps?: number;
}

export default function CameraFeed({
  rosbridgeUrl,
  topic = "/image_raw/compressed",
  mjpegUrl,
  maxFps = 24,
}: CameraFeedProps) {
  const imgRef = useRef<HTMLImageElement>(null);
  const [connected, setConnected] = useState(false);
  const [error, setError] = useState(false);

  // Default rosbridge URL: same host as page, port 9091
  const resolvedUrl = rosbridgeUrl
    || `ws://${window.location.hostname || "localhost"}:9091`;

  useEffect(() => {
    // If mjpegUrl provided, just use <img> directly — no WS needed
    if (mjpegUrl) {
      setConnected(true);
      return;
    }

    let ws: WebSocket | null = null;
    let lastFrame = 0;
    const minInterval = 1000 / maxFps;
    let objectUrl: string | null = null;

    function connect() {
      ws = new WebSocket(resolvedUrl);

      ws.onopen = () => {
        setConnected(true);
        setError(false);
        // Subscribe to compressed image topic
        ws!.send(JSON.stringify({
          op: "subscribe",
          id: "camera_feed",
          topic,
          type: "sensor_msgs/CompressedImage",
          throttle_rate: Math.round(minInterval),
          queue_length: 1,
        }));
      };

      ws.onmessage = (ev) => {
        const now = performance.now();
        if (now - lastFrame < minInterval) return;
        lastFrame = now;

        try {
          const msg = JSON.parse(ev.data);
          if (msg.op === "publish" && msg.msg?.data) {
            // msg.msg.data is base64-encoded JPEG
            const src = `data:image/jpeg;base64,${msg.msg.data}`;
            if (imgRef.current) {
              // Revoke previous object URL if we created one
              if (objectUrl) URL.revokeObjectURL(objectUrl);
              imgRef.current.src = src;
            }
          }
        } catch {
          // ignore parse errors
        }
      };

      ws.onerror = () => setError(true);

      ws.onclose = () => {
        setConnected(false);
        // Reconnect after 3s
        setTimeout(connect, 3000);
      };
    }

    connect();

    return () => {
      if (ws) {
        ws.onclose = null; // prevent reconnect on cleanup
        ws.close();
      }
      if (objectUrl) URL.revokeObjectURL(objectUrl);
    };
  }, [resolvedUrl, topic, mjpegUrl, maxFps]);

  return (
    <div className="camera-feed">
      <div className="camera-feed-header">
        <span className="camera-feed-label">CAM</span>
        <span
          className="camera-feed-dot"
          style={{
            background: error ? "#ef4444" : connected ? "#22c55e" : "#eab308",
          }}
        />
      </div>
      {mjpegUrl ? (
        <img
          ref={imgRef}
          src={mjpegUrl}
          alt="Robot camera"
          className="camera-feed-img"
          onError={() => setError(true)}
        />
      ) : (
        <img
          ref={imgRef}
          alt="Robot camera"
          className="camera-feed-img"
          style={!connected ? { opacity: 0.3 } : undefined}
        />
      )}
      {error && (
        <div className="camera-feed-offline">NO SIGNAL</div>
      )}
    </div>
  );
}
