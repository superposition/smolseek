"""
Mock Jetson WebSocket server for integration testing.

Simulates the Jetson point_cloud_ws:
  - Accepts WS connections on :9090
  - Sends a fake point cloud frame (0x01) on connect
  - Receives nav_goal JSON text → simulates navigation:
      navigating (decreasing distance) → arrived
"""

import asyncio
import json
import logging
import struct

import websockets
from websockets.server import serve

log = logging.getLogger("mock_jetson")

NAV_DELAY = 0.5   # seconds between nav status updates
NAV_STEPS = 3     # how many "navigating" frames before "arrived"


class MockJetson:
    def __init__(self, host: str = "0.0.0.0", port: int = 9090):
        self.host = host
        self.port = port
        self._clients: set = set()
        self._server = None
        self.goals_received: list[dict] = []
        self.nav_fail_mode: str | None = None  # set to "failed"/"timeout" to test fallbacks

    async def start(self):
        self._server = await serve(self._handler, self.host, self.port)
        log.info(f"Mock Jetson WS listening on :{self.port}")

    async def stop(self):
        if self._server:
            self._server.close()
            await self._server.wait_closed()

    async def _handler(self, ws):
        self._clients.add(ws)
        log.info(f"Client connected (total: {len(self._clients)})")

        # Send a tiny fake point cloud on connect (0x01 header + 3 floats)
        fake_cloud = struct.pack("B", 0x01) + struct.pack("<3f", 0.0, 0.0, 0.0)
        await ws.send(fake_cloud)

        try:
            async for message in ws:
                if isinstance(message, str):
                    await self._handle_text(ws, message)
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self._clients.discard(ws)

    async def _handle_text(self, ws, raw: str):
        try:
            data = json.loads(raw)
        except json.JSONDecodeError:
            return

        if data.get("type") == "nav_goal":
            self.goals_received.append(data)
            cache_id = data.get("cache_id", "")
            log.info(f"Received nav_goal for {cache_id}")
            asyncio.ensure_future(self._simulate_navigation(ws, cache_id))

    async def _simulate_navigation(self, ws, cache_id: str):
        """Send nav_status frames simulating robot movement."""
        total_distance = 1.5

        for i in range(NAV_STEPS):
            remaining = total_distance * (1 - (i + 1) / (NAV_STEPS + 1))
            status_data = {
                "status": "navigating",
                "cache_id": cache_id,
                "distance_remaining": round(remaining, 2),
            }
            frame = struct.pack("B", 0x14) + json.dumps(status_data).encode()
            await self._broadcast(frame)
            await asyncio.sleep(NAV_DELAY)

        # Final status
        if self.nav_fail_mode:
            final_status = self.nav_fail_mode
        else:
            final_status = "arrived"

        status_data = {
            "status": final_status,
            "cache_id": cache_id,
            "distance_remaining": 0.0,
        }
        frame = struct.pack("B", 0x14) + json.dumps(status_data).encode()
        await self._broadcast(frame)
        log.info(f"Nav complete: {final_status} for {cache_id}")

    async def _broadcast(self, data: bytes):
        stale = set()
        for ws in self._clients:
            try:
                await ws.send(data)
            except websockets.exceptions.ConnectionClosed:
                stale.add(ws)
        self._clients -= stale
