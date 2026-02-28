"""
E1: End-to-end integration test.

Full pipeline: mock Jetson + mock Unlink + coordinator + WS server.
Simulates two browser clients playing through 3 rounds.

Usage:
    python -m game.tests.integration_test          # fast (mock services)
    python -m game.tests.integration_test --live    # use real Jetson + Unlink
"""

from __future__ import annotations

import argparse
import asyncio
import json
import logging
import struct
import sys
import time
from typing import Optional

import websockets

# Patch config BEFORE importing coordinator (need fast timers for tests)
from game.coordinator import config

_ORIGINAL_JETSON_HOST = config.JETSON_WS_HOST
config.DEPOSIT_DURATION_SEC = 2
config.BID_DURATION_SEC = 4
config.NAV_TIMEOUT_SEC = 10

from game.coordinator.coordinator import Coordinator
from game.coordinator.game_ws_server import GameWSServer
from game.tests.mock_jetson import MockJetson
from game.tests.mock_unlink import MockUnlink

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(name)s] %(levelname)s %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("integration")

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

class PlayerClient:
    """Simulates a browser WS client."""

    def __init__(self, name: str, address: str, port: int = 8081):
        self.name = name
        self.address = address
        self.port = port
        self.ws: Optional[websockets.WebSocketClientProtocol] = None
        self.game_states: list[dict] = []
        self.round_results: list[dict] = []
        self.bid_results: list[dict] = []
        self.nav_statuses: list[dict] = []
        self._recv_task: Optional[asyncio.Task] = None

    async def connect(self):
        self.ws = await websockets.connect(f"ws://localhost:{self.port}")
        self._recv_task = asyncio.ensure_future(self._recv_loop())
        log.info(f"[{self.name}] Connected to game server")

    async def disconnect(self):
        if self._recv_task:
            self._recv_task.cancel()
        if self.ws:
            await self.ws.close()

    async def _recv_loop(self):
        try:
            async for msg in self.ws:
                if isinstance(msg, bytes) and len(msg) > 1:
                    msg_type = msg[0]
                    payload = msg[1:]
                    if msg_type == 0x10:  # GAME_STATE
                        data = json.loads(payload)
                        self.game_states.append(data)
                    elif msg_type == 0x11:  # CACHE_LOCS
                        pass  # handled via game_state
                    elif msg_type == 0x13:  # ROUND_RESULT
                        data = json.loads(payload)
                        self.round_results.append(data)
                        log.info(f"[{self.name}] Round result: winner={data.get('winner_id')}")
                    elif msg_type == 0x14:  # NAV_STATUS
                        data = json.loads(payload)
                        self.nav_statuses.append(data)
                elif isinstance(msg, str):
                    data = json.loads(msg)
                    if data.get("type") == "bid_result":
                        self.bid_results.append(data)
                        log.info(f"[{self.name}] Bid result: confirmed={data.get('confirmed')}")
                    elif data.get("type") == "joined":
                        log.info(f"[{self.name}] Join confirmed")
        except (websockets.exceptions.ConnectionClosed, asyncio.CancelledError):
            pass

    async def join(self):
        await self.ws.send(json.dumps({
            "type": "join",
            "playerId": self.name,
            "address": self.address,
        }))
        await asyncio.sleep(0.2)

    async def start_game(self):
        await self.ws.send(json.dumps({"type": "start"}))
        log.info(f"[{self.name}] Sent start command")

    async def bid(self, cache_id: str, amount: str):
        relay_id = f"0x_test_relay_{self.name}_{int(time.time()*1000)}"
        await self.ws.send(json.dumps({
            "type": "bid",
            "cacheId": cache_id,
            "relayId": relay_id,
            "amount": amount,
        }))
        log.info(f"[{self.name}] Bid {amount} on {cache_id}")

    @property
    def latest_state(self) -> Optional[dict]:
        return self.game_states[-1] if self.game_states else None

    def wait_for_phase(self, phase: str, timeout: float = 30) -> asyncio.Task:
        """Return a coroutine that waits until the latest game state matches phase."""
        async def _wait():
            deadline = time.time() + timeout
            while time.time() < deadline:
                if self.latest_state and self.latest_state.get("phase") == phase:
                    return True
                await asyncio.sleep(0.2)
            return False
        return _wait()


# ---------------------------------------------------------------------------
# Test scenario
# ---------------------------------------------------------------------------

async def run_test(live: bool = False):
    mock_jetson = None
    mock_unlink = None

    try:
        # ---- Step 1: Start services ----
        if not live:
            log.info("=" * 60)
            log.info("Starting mock services...")
            log.info("=" * 60)

            # Point coordinator at local mocks
            config.JETSON_WS_HOST = "127.0.0.1"

            mock_jetson = MockJetson()
            await mock_jetson.start()

            mock_unlink = MockUnlink()
            await mock_unlink.start()
        else:
            log.info("LIVE mode — using real Jetson + Unlink service")
            config.JETSON_WS_HOST = _ORIGINAL_JETSON_HOST

        # ---- Step 2: Start coordinator + WS server ----
        log.info("Starting game coordinator + WS server...")
        coordinator = Coordinator()
        server = GameWSServer(coordinator)

        # Run server in background
        server_task = asyncio.ensure_future(server.run())
        await asyncio.sleep(1)  # let server bind

        # ---- Step 3: Connect players ----
        log.info("=" * 60)
        log.info("Connecting players...")
        log.info("=" * 60)

        alice = PlayerClient("alice", "0xAlice_0001")
        bob = PlayerClient("bob", "0xBob_0002")
        await alice.connect()
        await bob.connect()

        # ---- Step 4: Join ----
        await alice.join()
        await bob.join()
        await asyncio.sleep(0.5)

        assert "alice" in coordinator.state.players, "Alice should be in players"
        assert "bob" in coordinator.state.players, "Bob should be in players"
        log.info("PASS: Both players joined")

        # ---- Step 5: Start game ----
        log.info("=" * 60)
        log.info("Starting game...")
        log.info("=" * 60)
        await alice.start_game()

        # Wait for DEPOSIT phase
        ok = await alice.wait_for_phase("DEPOSIT", timeout=5)
        assert ok, "Should reach DEPOSIT phase"
        log.info("PASS: DEPOSIT phase reached")

        # ---- Step 6: Wait for PLAY phase (deposit timer is 2s in test) ----
        ok = await alice.wait_for_phase("PLAY", timeout=10)
        assert ok, "Should reach PLAY phase"
        assert coordinator.state.current_round == 1, "Should be round 1"
        log.info("PASS: PLAY phase, round 1")

        # ---- Step 7: Play 3 rounds ----
        caches = list(coordinator.state.caches.keys())

        for round_num in range(1, 4):
            log.info("=" * 60)
            log.info(f"ROUND {round_num}")
            log.info("=" * 60)

            # Wait for round to start
            deadline = time.time() + 10
            while coordinator.state.current_round < round_num and time.time() < deadline:
                await asyncio.sleep(0.2)
            assert coordinator.state.current_round == round_num, f"Expected round {round_num}"

            # Pick target cache
            remaining = coordinator.state.remaining_caches()
            target = remaining[0]
            log.info(f"Target cache: {target.id} (value={target.value})")

            # Both players bid — alice bids higher
            alice_amount = str(200000000000000000 * round_num)
            bob_amount = str(100000000000000000 * round_num)

            await alice.bid(target.id, alice_amount)
            await bob.bid(target.id, bob_amount)

            # Wait for bid confirmation
            await asyncio.sleep(1)
            log.info(f"Alice bid results: {len(alice.bid_results)}, Bob: {len(bob.bid_results)}")

            # Wait for bidding to end + nav to complete
            deadline = time.time() + 20
            while time.time() < deadline:
                rnd = coordinator.state.active_round
                if rnd and rnd.number == round_num and rnd.ended_at:
                    break
                await asyncio.sleep(0.3)

            rnd = coordinator.state.rounds[round_num - 1]
            log.info(f"Round {round_num} ended: winner={rnd.winner_id} nav={rnd.nav_status}")

            # Verify round results
            assert rnd.winner_id == "alice", f"Alice should win (bid higher), got {rnd.winner_id}"
            assert rnd.target_cache_id == target.id
            log.info(f"PASS: Round {round_num} — alice won, cache {target.id} collected")

            # Verify cache marked collected
            cache = coordinator.state.caches[target.id]
            assert cache.collected, f"Cache {target.id} should be collected"
            assert cache.collected_by == "alice"
            log.info(f"PASS: Cache {target.id} marked collected by alice")

            # Brief pause between rounds
            await asyncio.sleep(0.5)

        # ---- Step 8: Verify scores ----
        log.info("=" * 60)
        log.info("Verifying final state...")
        log.info("=" * 60)

        alice_player = coordinator.state.players["alice"]
        bob_player = coordinator.state.players["bob"]
        log.info(f"Alice score: {alice_player.score} wei")
        log.info(f"Bob score: {bob_player.score} wei")
        assert alice_player.score > 0, "Alice should have score > 0"
        assert bob_player.score == 0, "Bob should have score 0 (never won)"

        # Verify leaderboard
        lb = coordinator.state.leaderboard()
        assert lb[0]["id"] == "alice", "Alice should be #1"
        log.info(f"PASS: Leaderboard — {lb}")

        # ---- Step 9: Check nav statuses were received ----
        assert len(alice.nav_statuses) > 0, "Should have received nav_status frames"
        log.info(f"PASS: Received {len(alice.nav_statuses)} nav_status updates")

        # ---- Step 10: Check mock services ----
        if mock_jetson:
            assert len(mock_jetson.goals_received) >= 3, "Jetson should have received 3 nav goals"
            log.info(f"PASS: Mock Jetson received {len(mock_jetson.goals_received)} nav goals")

        # ---- Done ----
        log.info("=" * 60)
        log.info("ALL TESTS PASSED")
        log.info("=" * 60)

        await alice.disconnect()
        await bob.disconnect()
        server_task.cancel()

    except AssertionError as e:
        log.error(f"ASSERTION FAILED: {e}")
        sys.exit(1)
    except Exception as e:
        log.error(f"TEST ERROR: {e}", exc_info=True)
        sys.exit(1)
    finally:
        if mock_jetson:
            await mock_jetson.stop()
        if mock_unlink:
            await mock_unlink.stop()


# ---------------------------------------------------------------------------
# Fallback scenario tests
# ---------------------------------------------------------------------------

async def test_no_bids():
    """Round with no bids should be skipped."""
    log.info("=" * 60)
    log.info("FALLBACK TEST: No bids in round")
    log.info("=" * 60)

    config.JETSON_WS_HOST = "127.0.0.1"
    mock_jetson = MockJetson()
    await mock_jetson.start()
    mock_unlink = MockUnlink()
    await mock_unlink.start()

    try:
        coordinator = Coordinator()
        server = GameWSServer(coordinator)
        server_task = asyncio.ensure_future(server.run())
        await asyncio.sleep(1)

        alice = PlayerClient("alice", "0xAlice")
        await alice.connect()
        await alice.join()
        await alice.start_game()

        # Wait for PLAY
        ok = await alice.wait_for_phase("PLAY", timeout=10)
        assert ok, "Should reach PLAY"

        # Don't bid — wait for round to end (4s bid timer)
        await asyncio.sleep(6)
        rnd = coordinator.state.rounds[0]
        assert rnd.winner_id is None, "No winner when no bids"
        assert rnd.ended_at is not None, "Round should have ended"
        log.info("PASS: No-bid round skipped correctly")

        await alice.disconnect()
        server_task.cancel()
    finally:
        await mock_jetson.stop()
        await mock_unlink.stop()


async def test_nav_failure():
    """Robot navigation failure should skip round."""
    log.info("=" * 60)
    log.info("FALLBACK TEST: Navigation failure")
    log.info("=" * 60)

    config.JETSON_WS_HOST = "127.0.0.1"
    mock_jetson = MockJetson()
    mock_jetson.nav_fail_mode = "failed"
    await mock_jetson.start()
    mock_unlink = MockUnlink()
    await mock_unlink.start()

    try:
        coordinator = Coordinator()
        server = GameWSServer(coordinator)
        server_task = asyncio.ensure_future(server.run())
        await asyncio.sleep(1)

        alice = PlayerClient("alice", "0xAlice")
        await alice.connect()
        await alice.join()
        await alice.start_game()

        ok = await alice.wait_for_phase("PLAY", timeout=10)
        assert ok

        # Bid on first cache
        target = list(coordinator.state.caches.keys())[0]
        await alice.bid(target, "100000000000000000")
        await asyncio.sleep(8)  # bid timer + nav simulation

        rnd = coordinator.state.rounds[0]
        assert rnd.nav_status == "failed", f"Expected 'failed', got {rnd.nav_status}"
        cache = coordinator.state.caches[target]
        assert not cache.collected, "Cache should NOT be collected on nav failure"
        assert alice.latest_state["phase"] == "PLAY", "Game should continue to next round"
        log.info("PASS: Navigation failure handled — round skipped, game continues")

        await alice.disconnect()
        server_task.cancel()
    finally:
        await mock_jetson.stop()
        await mock_unlink.stop()


async def test_nav_timeout():
    """Robot timeout should skip round."""
    log.info("=" * 60)
    log.info("FALLBACK TEST: Navigation timeout")
    log.info("=" * 60)

    config.JETSON_WS_HOST = "127.0.0.1"
    mock_jetson = MockJetson()
    await mock_jetson.start()
    mock_unlink = MockUnlink()
    await mock_unlink.start()

    # Override nav timeout to be very short for this test
    old_timeout = config.NAV_TIMEOUT_SEC
    config.NAV_TIMEOUT_SEC = 3

    try:
        coordinator = Coordinator()
        server = GameWSServer(coordinator)
        server_task = asyncio.ensure_future(server.run())
        await asyncio.sleep(1)

        # Make mock jetson not send any nav responses
        mock_jetson.nav_fail_mode = None
        # Monkey-patch to suppress nav simulation entirely
        mock_jetson._simulate_navigation = lambda *a, **k: asyncio.sleep(999)

        alice = PlayerClient("alice", "0xAlice")
        await alice.connect()
        await alice.join()
        await alice.start_game()

        ok = await alice.wait_for_phase("PLAY", timeout=10)
        assert ok

        target = list(coordinator.state.caches.keys())[0]
        await alice.bid(target, "100000000000000000")
        await asyncio.sleep(10)  # bid + timeout

        rnd = coordinator.state.rounds[0]
        assert rnd.nav_status == "timeout", f"Expected 'timeout', got {rnd.nav_status}"
        log.info("PASS: Navigation timeout handled correctly")

        await alice.disconnect()
        server_task.cancel()
    finally:
        config.NAV_TIMEOUT_SEC = old_timeout
        await mock_jetson.stop()
        await mock_unlink.stop()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

async def main():
    parser = argparse.ArgumentParser(description="smolseek E2E integration test")
    parser.add_argument("--live", action="store_true", help="Use real Jetson + Unlink")
    parser.add_argument("--fallbacks", action="store_true", help="Run fallback scenario tests")
    args = parser.parse_args()

    if args.fallbacks:
        await test_no_bids()
        await test_nav_failure()
        await test_nav_timeout()
        log.info("=" * 60)
        log.info("ALL FALLBACK TESTS PASSED")
        log.info("=" * 60)
    else:
        await run_test(live=args.live)


if __name__ == "__main__":
    asyncio.run(main())
