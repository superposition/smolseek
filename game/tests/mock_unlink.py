"""
Mock Unlink payment service for integration testing.

Replaces the real unlink-service (Express + @unlink-xyz/node) with a
lightweight aiohttp server that auto-confirms bids and distributions.
"""

import logging
from aiohttp import web

log = logging.getLogger("mock_unlink")

MOCK_ESCROW_ADDRESS = "0xMOCK_ESCROW_000000000000000000000000"
MOCK_BALANCE = "10000000000000000000"  # 10 MON

_bids: list[dict] = []
_distributions: list[dict] = []


async def health(_request):
    return web.json_response({"status": "ok", "mock": True})


async def escrow_address(_request):
    return web.json_response({"address": MOCK_ESCROW_ADDRESS})


async def balance(_request):
    return web.json_response({"balance": MOCK_BALANCE})


async def game_pool(_request):
    return web.json_response({"address": MOCK_ESCROW_ADDRESS, "balance": MOCK_BALANCE})


async def sync_handler(_request):
    return web.json_response({"synced": True})


async def post_bid(request):
    data = await request.json()
    relay_id = data.get("relayId", "")
    log.info(f"Mock bid verify: relay={relay_id}")

    bid_record = {
        "relayId": relay_id,
        "playerId": data.get("playerId"),
        "cacheId": data.get("cacheId"),
        "round": data.get("round"),
        "confirmed": True,
    }
    _bids.append(bid_record)

    return web.json_response({
        "confirmed": True,
        "amount": "100000000000000000",
        "relayId": relay_id,
        "bidId": len(_bids),
    })


async def get_bids(request):
    round_num = int(request.match_info["round"])
    round_bids = [b for b in _bids if b.get("round") == round_num]
    return web.json_response(round_bids)


async def post_distribute(request):
    data = await request.json()
    recipient = data.get("recipientAddress", "")
    amount = data.get("amount", "0")
    round_num = data.get("round", 0)
    log.info(f"Mock distribute: {amount} → {recipient} (round {round_num})")

    # Check for double-send
    for d in _distributions:
        if d["round"] == round_num and d["recipient"] == recipient:
            return web.json_response({"error": "already distributed"}, status=409)

    _distributions.append({"recipient": recipient, "amount": amount, "round": round_num})
    return web.json_response({"relayId": f"mock_relay_{round_num}", "status": "sent"})


class MockUnlink:
    def __init__(self, host: str = "0.0.0.0", port: int = 3001):
        self.host = host
        self.port = port
        self._runner = None

    async def start(self):
        app = web.Application()
        app.router.add_get("/health", health)
        app.router.add_get("/escrow-address", escrow_address)
        app.router.add_get("/balance", balance)
        app.router.add_get("/balance/game-pool", game_pool)
        app.router.add_get("/sync", sync_handler)
        app.router.add_post("/bid", post_bid)
        app.router.add_get("/bid/{round}", get_bids)
        app.router.add_post("/distribute", post_distribute)

        self._runner = web.AppRunner(app)
        await self._runner.setup()
        site = web.TCPSite(self._runner, self.host, self.port)
        await site.start()
        log.info(f"Mock Unlink service on :{self.port}")

    async def stop(self):
        if self._runner:
            await self._runner.cleanup()

    @staticmethod
    def reset():
        _bids.clear()
        _distributions.clear()
