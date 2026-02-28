"""Game data models."""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional


class Phase(str, Enum):
    LOBBY = "LOBBY"
    SETUP = "SETUP"
    DEPOSIT = "DEPOSIT"
    PLAY = "PLAY"
    END = "END"


@dataclass
class Player:
    id: str                        # Pseudonymous player ID
    address: Optional[str] = None  # Unlink wallet address (for distributions)
    score: int = 0                 # Total MON wei earned
    connected: bool = True

    def to_dict(self) -> dict:
        return {
            "id": self.id,
            "address": self.address,
            "score": self.score,
            "connected": self.connected,
        }


@dataclass
class Cache:
    id: str
    x: float
    y: float
    z: float = 0.0
    value: str = "0"              # MON wei
    collected: bool = False
    collected_by: Optional[str] = None

    def to_dict(self) -> dict:
        return {
            "id": self.id,
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "value": self.value,
            "collected": self.collected,
            "collected_by": self.collected_by,
        }


@dataclass
class Bid:
    player_id: str
    cache_id: str
    amount: str                   # MON wei (string for big ints)
    relay_id: str
    confirmed: bool = False
    timestamp: float = field(default_factory=time.time)

    def amount_int(self) -> int:
        return int(self.amount) if self.amount else 0

    def to_dict(self) -> dict:
        return {
            "player_id": self.player_id,
            "cache_id": self.cache_id,
            "amount": self.amount,
            "relay_id": self.relay_id,
            "confirmed": self.confirmed,
        }


@dataclass
class Round:
    number: int
    bids: list[Bid] = field(default_factory=list)
    winner_id: Optional[str] = None
    winning_bid: Optional[Bid] = None
    target_cache_id: Optional[str] = None
    nav_status: Optional[str] = None      # navigating, arrived, failed, timeout
    started_at: float = field(default_factory=time.time)
    ended_at: Optional[float] = None

    def confirmed_bids(self) -> list[Bid]:
        return [b for b in self.bids if b.confirmed]

    def highest_bid(self) -> Optional[Bid]:
        confirmed = self.confirmed_bids()
        if not confirmed:
            return None
        # Sort by amount descending, tie-break by timestamp (first wins)
        confirmed.sort(key=lambda b: (-b.amount_int(), b.timestamp))
        return confirmed[0]

    def to_dict(self) -> dict:
        return {
            "number": self.number,
            "bids": [b.to_dict() for b in self.confirmed_bids()],
            "winner_id": self.winner_id,
            "target_cache_id": self.target_cache_id,
            "nav_status": self.nav_status,
        }


@dataclass
class GameState:
    phase: Phase = Phase.LOBBY
    players: dict[str, Player] = field(default_factory=dict)
    caches: dict[str, Cache] = field(default_factory=dict)
    rounds: list[Round] = field(default_factory=list)
    current_round: int = 0
    phase_deadline: Optional[float] = None  # Unix timestamp for current phase/round end

    @property
    def active_round(self) -> Optional[Round]:
        if self.rounds and self.current_round > 0:
            return self.rounds[self.current_round - 1]
        return None

    def remaining_caches(self) -> list[Cache]:
        return [c for c in self.caches.values() if not c.collected]

    def leaderboard(self) -> list[dict]:
        """Players sorted by score descending."""
        players = sorted(self.players.values(), key=lambda p: -p.score)
        return [
            {"rank": i + 1, "id": p.id, "score": p.score}
            for i, p in enumerate(players)
        ]

    def to_dict(self) -> dict:
        return {
            "phase": self.phase.value,
            "current_round": self.current_round,
            "total_rounds": len(self.caches),
            "players": {pid: p.to_dict() for pid, p in self.players.items()},
            "caches": {cid: c.to_dict() for cid, c in self.caches.items()},
            "leaderboard": self.leaderboard(),
            "phase_deadline": self.phase_deadline,
        }
