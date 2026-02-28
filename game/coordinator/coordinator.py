"""
Game Coordinator — FSM driving the OpenClaw game phases.

Phases: LOBBY → SETUP → DEPOSIT → PLAY → END

The coordinator owns the GameState and exposes methods that the WS server
calls in response to player actions and timer events.
"""

from __future__ import annotations

import asyncio
import logging
import time
from typing import Callable, Optional, Awaitable

import aiohttp

from . import config
from .models import Bid, Cache, GameState, Phase, Player, Round

log = logging.getLogger("coordinator")


class Coordinator:
    """Finite state machine for the OpenClaw game."""

    def __init__(self):
        self.state = GameState()
        self._timer_task: Optional[asyncio.Task] = None

        # Callbacks set by the WS server
        self.on_state_changed: Optional[Callable[[], Awaitable[None]]] = None
        self.on_dispatch_robot: Optional[Callable[[Cache], Awaitable[None]]] = None
        self.on_round_result: Optional[Callable[[Round], Awaitable[None]]] = None

    # ------------------------------------------------------------------
    # Phase transitions
    # ------------------------------------------------------------------
    async def _set_phase(self, phase: Phase, deadline: float | None = None):
        self.state.phase = phase
        self.state.phase_deadline = deadline
        log.info(f"Phase → {phase.value}" + (f" (deadline in {deadline - time.time():.0f}s)" if deadline else ""))
        if self.on_state_changed:
            await self.on_state_changed()

    # ---- LOBBY ----
    async def player_join(self, player_id: str, address: str | None = None) -> Player:
        """Add a player to the lobby."""
        if player_id in self.state.players:
            p = self.state.players[player_id]
            p.connected = True
            log.info(f"Player reconnected: {player_id}")
        else:
            p = Player(id=player_id, address=address)
            self.state.players[player_id] = p
            log.info(f"Player joined: {player_id} (total: {len(self.state.players)})")
        if self.on_state_changed:
            await self.on_state_changed()
        return p

    def player_disconnect(self, player_id: str):
        if player_id in self.state.players:
            self.state.players[player_id].connected = False
            log.info(f"Player disconnected: {player_id}")

    async def host_start_game(self):
        """Host triggers game start from LOBBY."""
        if self.state.phase != Phase.LOBBY:
            log.warning(f"Cannot start: phase is {self.state.phase.value}")
            return
        await self._enter_setup()

    # ---- SETUP ----
    async def _enter_setup(self):
        """Place caches (hardcoded for now)."""
        await self._set_phase(Phase.SETUP)
        for c in config.DEFAULT_CACHES:
            cache = Cache(**c)
            self.state.caches[cache.id] = cache
        log.info(f"Placed {len(self.state.caches)} caches")
        # Auto-advance to DEPOSIT
        await self._enter_deposit()

    # ---- DEPOSIT ----
    async def _enter_deposit(self):
        """Players have config.DEPOSIT_DURATION_SEC to fund escrow."""
        deadline = time.time() + config.DEPOSIT_DURATION_SEC
        await self._set_phase(Phase.DEPOSIT, deadline)
        self._start_timer(config.DEPOSIT_DURATION_SEC, self._enter_play)

    # ---- PLAY ----
    async def _enter_play(self):
        """Start the play phase — begin first round."""
        await self._set_phase(Phase.PLAY)
        await self._start_round()

    async def _start_round(self):
        """Begin a new bidding round."""
        remaining = self.state.remaining_caches()
        if not remaining:
            log.info("No caches remaining — ending game")
            await self._enter_end()
            return

        self.state.current_round += 1
        rnd = Round(number=self.state.current_round)
        self.state.rounds.append(rnd)

        deadline = time.time() + config.BID_DURATION_SEC
        self.state.phase_deadline = deadline

        log.info(
            f"Round {rnd.number} started — "
            f"{len(remaining)} caches remaining, "
            f"bidding open for {config.BID_DURATION_SEC}s"
        )
        if self.on_state_changed:
            await self.on_state_changed()

        self._start_timer(config.BID_DURATION_SEC, self._end_bidding)

    async def _end_bidding(self):
        """Bidding time expired — pick winner and dispatch robot."""
        rnd = self.state.active_round
        if rnd is None:
            return

        winner = rnd.highest_bid()
        if winner is None:
            log.info(f"Round {rnd.number}: no bids — skipping")
            rnd.ended_at = time.time()
            if self.on_round_result:
                await self.on_round_result(rnd)
            await self._next_round_or_end()
            return

        rnd.winner_id = winner.player_id
        rnd.winning_bid = winner
        rnd.target_cache_id = winner.cache_id

        cache = self.state.caches.get(winner.cache_id)
        if cache is None or cache.collected:
            log.warning(f"Round {rnd.number}: target cache {winner.cache_id} invalid/collected")
            rnd.ended_at = time.time()
            await self._next_round_or_end()
            return

        log.info(
            f"Round {rnd.number}: winner={winner.player_id} "
            f"bid={winner.amount} cache={winner.cache_id}"
        )

        # Dispatch robot to cache
        rnd.nav_status = "dispatching"
        if self.on_state_changed:
            await self.on_state_changed()

        if self.on_dispatch_robot:
            await self.on_dispatch_robot(cache)

        # Start nav timeout
        self._start_timer(config.NAV_TIMEOUT_SEC, self._nav_timeout)

    async def handle_nav_status(self, status: str, cache_id: str, distance: float):
        """Called by WS server when nav_status arrives from Jetson."""
        rnd = self.state.active_round
        if rnd is None:
            return

        rnd.nav_status = status
        log.info(f"Nav status: {status} cache={cache_id} dist={distance:.2f}")

        if status == "arrived":
            self._cancel_timer()
            await self._cache_collected(rnd)
        elif status in ("failed", "timeout", "aborted"):
            self._cancel_timer()
            log.warning(f"Navigation {status} for cache {cache_id}")
            rnd.ended_at = time.time()
            if self.on_round_result:
                await self.on_round_result(rnd)
            if self.on_state_changed:
                await self.on_state_changed()
            await self._next_round_or_end()

    async def _nav_timeout(self):
        """Robot didn't arrive in time."""
        rnd = self.state.active_round
        if rnd is None:
            return
        log.warning(f"Round {rnd.number}: nav timeout")
        rnd.nav_status = "timeout"
        rnd.ended_at = time.time()
        if self.on_round_result:
            await self.on_round_result(rnd)
        await self._next_round_or_end()

    async def _cache_collected(self, rnd: Round):
        """Robot arrived — award cache value to winner."""
        cache = self.state.caches.get(rnd.target_cache_id)
        if cache is None:
            return

        cache.collected = True
        cache.collected_by = rnd.winner_id

        # Award score
        if rnd.winner_id and rnd.winner_id in self.state.players:
            player = self.state.players[rnd.winner_id]
            player.score += int(cache.value)
            log.info(
                f"Cache {cache.id} collected by {rnd.winner_id} — "
                f"awarded {cache.value} (total score: {player.score})"
            )

            # Distribute winnings via Unlink service
            if player.address:
                await self._distribute_winnings(player.address, cache.value, rnd.number)

        rnd.ended_at = time.time()
        if self.on_round_result:
            await self.on_round_result(rnd)
        if self.on_state_changed:
            await self.on_state_changed()
        await self._next_round_or_end()

    async def _next_round_or_end(self):
        """Start next round or end game if no caches remain."""
        remaining = self.state.remaining_caches()
        if not remaining:
            await self._enter_end()
        else:
            # Brief pause between rounds
            await asyncio.sleep(3)
            await self._start_round()

    # ---- END ----
    async def _enter_end(self):
        await self._set_phase(Phase.END)
        log.info("Game over!")
        log.info(f"Final leaderboard: {self.state.leaderboard()}")

    # ------------------------------------------------------------------
    # Bid handling
    # ------------------------------------------------------------------
    async def submit_bid(
        self, player_id: str, cache_id: str, relay_id: str, amount: str
    ) -> dict:
        """Process a bid from a player. Verifies via Unlink service."""
        rnd = self.state.active_round
        if rnd is None or self.state.phase != Phase.PLAY:
            return {"error": "Not in play phase"}

        # Validate cache
        cache = self.state.caches.get(cache_id)
        if cache is None or cache.collected:
            return {"error": f"Cache {cache_id} invalid or already collected"}

        # Validate player
        if player_id not in self.state.players:
            return {"error": f"Unknown player {player_id}"}

        bid = Bid(
            player_id=player_id,
            cache_id=cache_id,
            amount=amount,
            relay_id=relay_id,
        )

        # Verify bid via Unlink service
        verified = await self._verify_bid(relay_id, player_id, cache_id, rnd.number)
        bid.confirmed = verified

        rnd.bids.append(bid)
        log.info(
            f"Bid: player={player_id} cache={cache_id} "
            f"amount={amount} confirmed={verified}"
        )

        if self.on_state_changed:
            await self.on_state_changed()

        return {"confirmed": verified, "bid": bid.to_dict()}

    # ------------------------------------------------------------------
    # Unlink service integration
    # ------------------------------------------------------------------
    async def _verify_bid(
        self, relay_id: str, player_id: str, cache_id: str, round_num: int
    ) -> bool:
        """Call Unlink service POST /bid to verify payment."""
        try:
            async with aiohttp.ClientSession() as session:
                async with session.post(
                    f"{config.UNLINK_SERVICE_URL}/bid",
                    json={
                        "relayId": relay_id,
                        "playerId": player_id,
                        "cacheId": cache_id,
                        "round": round_num,
                    },
                    timeout=aiohttp.ClientTimeout(total=35),
                ) as resp:
                    data = await resp.json()
                    return data.get("confirmed", False)
        except Exception as e:
            log.error(f"Bid verification failed: {e}")
            return False

    async def _distribute_winnings(
        self, recipient: str, amount: str, round_num: int
    ):
        """Call Unlink service POST /distribute to send winnings."""
        try:
            async with aiohttp.ClientSession() as session:
                async with session.post(
                    f"{config.UNLINK_SERVICE_URL}/distribute",
                    json={
                        "recipientAddress": recipient,
                        "amount": amount,
                        "round": round_num,
                    },
                    timeout=aiohttp.ClientTimeout(total=15),
                ) as resp:
                    data = await resp.json()
                    if resp.status == 200:
                        log.info(f"Distributed {amount} to {recipient}: {data}")
                    else:
                        log.warning(f"Distribution failed ({resp.status}): {data}")
        except Exception as e:
            log.error(f"Distribution error: {e}")

    # ------------------------------------------------------------------
    # Timer helpers
    # ------------------------------------------------------------------
    def _start_timer(self, seconds: float, callback):
        self._cancel_timer()

        async def _run():
            await asyncio.sleep(seconds)
            await callback()

        self._timer_task = asyncio.ensure_future(_run())

    def _cancel_timer(self):
        if self._timer_task and not self._timer_task.done():
            self._timer_task.cancel()
            self._timer_task = None
