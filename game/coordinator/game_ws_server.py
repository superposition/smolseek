"""
Game WebSocket Server — proxies Jetson map data to browsers + game messaging.

Upstream: connects to Jetson point_cloud_ws at :9090 as a WS client
Downstream: serves browser clients on :8081

Forwards binary map frames (0x01-0x04, 0x14) from Jetson to browsers.
Adds game message types (0x10-0x13).
Receives text commands from browsers (join, bid, start).
Sends nav_goal text to Jetson for robot dispatch.
"""

from __future__ import annotations

import asyncio
import json
import logging
import struct
import time
from typing import Optional

import websockets
from websockets.server import serve

from . import config
from .config import (
    MSG_CACHE_LOCS,
    MSG_GAME_STATE,
    MSG_NAV_STATUS,
    MSG_ROBOT_TARGET,
    MSG_ROUND_RESULT,
)
from .coordinator import Coordinator
from .models import Cache, Round

log = logging.getLogger("game_ws")


class GameWSServer:
    """WebSocket proxy + game message server."""

    def __init__(self, coordinator: Coordinator):
        self.coord = coordinator
        self._browser_clients: set[websockets.WebSocketServerProtocol] = set()
        self._jetson_ws: Optional[websockets.WebSocketClientProtocol] = None
        self._jetson_connected = False
        self._player_map: dict[websockets.WebSocketServerProtocol, str] = {}  # ws → player_id

        # Wire coordinator callbacks
        self.coord.on_state_changed = self._broadcast_game_state
        self.coord.on_dispatch_robot = self._dispatch_robot
        self.coord.on_round_result = self._broadcast_round_result

    # ------------------------------------------------------------------
    # Main entry point
    # ------------------------------------------------------------------
    async def run(self):
        """Start both upstream (Jetson) and downstream (browser) connections."""
        log.info(f"Game WS server starting on :{config.GAME_WS_PORT}")
        log.info(f"Upstream Jetson at ws://{config.JETSON_WS_HOST}:{config.JETSON_WS_PORT}")

        async with serve(self._browser_handler, "0.0.0.0", config.GAME_WS_PORT):
            # Run upstream connection and server concurrently
            await asyncio.gather(
                self._upstream_loop(),
                asyncio.Future(),  # keep server alive
            )

    # ------------------------------------------------------------------
    # Upstream: Jetson connection
    # ------------------------------------------------------------------
    async def _upstream_loop(self):
        """Maintain persistent connection to Jetson WS, reconnect on failure."""
        while True:
            try:
                uri = f"ws://{config.JETSON_WS_HOST}:{config.JETSON_WS_PORT}"
                log.info(f"Connecting to Jetson at {uri}...")
                async with websockets.connect(uri) as ws:
                    self._jetson_ws = ws
                    self._jetson_connected = True
                    log.info("Jetson upstream connected!")
                    await self._upstream_recv(ws)
            except (ConnectionRefusedError, OSError) as e:
                log.warning(f"Jetson connection failed: {e} — retrying in 3s")
            except websockets.exceptions.ConnectionClosed:
                log.warning("Jetson connection closed — retrying in 3s")
            finally:
                self._jetson_ws = None
                self._jetson_connected = False
            await asyncio.sleep(3)

    async def _upstream_recv(self, ws):
        """Receive frames from Jetson and forward to all browsers."""
        async for message in ws:
            if isinstance(message, bytes) and len(message) > 0:
                msg_type = message[0]

                # Forward map data to all browsers
                if msg_type in (0x01, 0x02, 0x03, 0x04):
                    await self._broadcast_binary(message)

                # Nav status (0x14) — forward to browsers AND feed to coordinator
                elif msg_type == MSG_NAV_STATUS:
                    await self._broadcast_binary(message)
                    payload = message[1:].decode("utf-8", errors="replace")
                    try:
                        data = json.loads(payload)
                        await self.coord.handle_nav_status(
                            data.get("status", ""),
                            data.get("cache_id", ""),
                            data.get("distance_remaining", 0.0),
                        )
                    except json.JSONDecodeError:
                        log.warning(f"Bad nav_status JSON: {payload}")

    async def _send_to_jetson(self, text: str):
        """Send a text message to the Jetson upstream WS."""
        if self._jetson_ws and self._jetson_connected:
            try:
                await self._jetson_ws.send(text)
                log.info(f"Sent to Jetson: {text}")
            except websockets.exceptions.ConnectionClosed:
                log.warning("Failed to send to Jetson — connection closed")
        else:
            log.warning("Cannot send to Jetson — not connected")

    # ------------------------------------------------------------------
    # Downstream: Browser clients
    # ------------------------------------------------------------------
    async def _browser_handler(self, websocket):
        """Handle a browser client connection."""
        self._browser_clients.add(websocket)
        client_addr = websocket.remote_address
        log.info(f"Browser connected: {client_addr} (total: {len(self._browser_clients)})")

        try:
            # Send current game state immediately
            await self._send_game_state(websocket)
            await self._send_cache_locations(websocket)

            # Listen for messages
            async for raw_msg in websocket:
                if isinstance(raw_msg, str):
                    await self._handle_browser_message(websocket, raw_msg)
        except websockets.exceptions.ConnectionClosed:
            log.info(f"Browser disconnected: {client_addr}")
        finally:
            # Clean up player mapping
            player_id = self._player_map.pop(websocket, None)
            if player_id:
                self.coord.player_disconnect(player_id)
            self._browser_clients.discard(websocket)

    async def _handle_browser_message(self, websocket, raw: str):
        """Route an incoming text message from a browser client."""
        try:
            data = json.loads(raw)
        except json.JSONDecodeError:
            log.warning(f"Bad JSON from browser: {raw}")
            return

        msg_type = data.get("type", "")
        log.info(f"Browser msg: type={msg_type} data={raw}")

        if msg_type == "join":
            await self._handle_join(websocket, data)
        elif msg_type == "start":
            await self._handle_start()
        elif msg_type == "bid":
            await self._handle_bid(websocket, data)
        else:
            log.warning(f"Unknown browser message type: {msg_type}")

    async def _handle_join(self, websocket, data: dict):
        """Player joins the game."""
        player_id = data.get("playerId", "")
        address = data.get("address")
        if not player_id:
            return

        self._player_map[websocket] = player_id
        player = await self.coord.player_join(player_id, address)

        # Send confirmation back
        await websocket.send(json.dumps({
            "type": "joined",
            "playerId": player.id,
        }))

    async def _handle_start(self):
        """Host starts the game."""
        await self.coord.host_start_game()

    async def _handle_bid(self, websocket, data: dict):
        """Player submits a bid."""
        player_id = self._player_map.get(websocket)
        if not player_id:
            await websocket.send(json.dumps({"type": "error", "error": "Not joined"}))
            return

        result = await self.coord.submit_bid(
            player_id=player_id,
            cache_id=data.get("cacheId", ""),
            relay_id=data.get("relayId", ""),
            amount=data.get("amount", "0"),
        )

        await websocket.send(json.dumps({"type": "bid_result", **result}))

    # ------------------------------------------------------------------
    # Coordinator callbacks
    # ------------------------------------------------------------------
    async def _broadcast_game_state(self):
        """Called by coordinator when state changes — push to all browsers."""
        state_json = json.dumps(self.coord.state.to_dict())
        frame = struct.pack("B", MSG_GAME_STATE) + state_json.encode("utf-8")
        await self._broadcast_binary(frame)

        # Also send cache locations
        cache_json = json.dumps(
            [c.to_dict() for c in self.coord.state.caches.values()]
        )
        cache_frame = struct.pack("B", MSG_CACHE_LOCS) + cache_json.encode("utf-8")
        await self._broadcast_binary(cache_frame)

    async def _dispatch_robot(self, cache: Cache):
        """Called by coordinator to send nav goal to Jetson."""
        goal = json.dumps({
            "type": "nav_goal",
            "x": cache.x,
            "y": cache.y,
            "theta": 0.0,
            "cache_id": cache.id,
        })
        await self._send_to_jetson(goal)

        # Broadcast robot target to browsers
        target_json = json.dumps({
            "cache_id": cache.id,
            "x": cache.x,
            "y": cache.y,
        })
        frame = struct.pack("B", MSG_ROBOT_TARGET) + target_json.encode("utf-8")
        await self._broadcast_binary(frame)

    async def _broadcast_round_result(self, rnd: Round):
        """Called by coordinator when a round ends — broadcast to browsers."""
        result_json = json.dumps(rnd.to_dict())
        frame = struct.pack("B", MSG_ROUND_RESULT) + result_json.encode("utf-8")
        await self._broadcast_binary(frame)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    async def _send_game_state(self, websocket):
        """Send current game state to a single client."""
        state_json = json.dumps(self.coord.state.to_dict())
        frame = struct.pack("B", MSG_GAME_STATE) + state_json.encode("utf-8")
        try:
            await websocket.send(frame)
        except websockets.exceptions.ConnectionClosed:
            pass

    async def _send_cache_locations(self, websocket):
        """Send cache locations to a single client."""
        cache_json = json.dumps(
            [c.to_dict() for c in self.coord.state.caches.values()]
        )
        frame = struct.pack("B", MSG_CACHE_LOCS) + cache_json.encode("utf-8")
        try:
            await websocket.send(frame)
        except websockets.exceptions.ConnectionClosed:
            pass

    async def _broadcast_binary(self, data: bytes):
        """Send binary frame to all connected browsers."""
        if not self._browser_clients:
            return
        stale = set()
        for ws in self._browser_clients:
            try:
                await ws.send(data)
            except websockets.exceptions.ConnectionClosed:
                stale.add(ws)
        self._browser_clients -= stale
