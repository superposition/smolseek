"""Entry point: python -m game.coordinator"""

import asyncio
import logging

from .coordinator import Coordinator
from .game_ws_server import GameWSServer

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(name)s] %(levelname)s %(message)s",
    datefmt="%H:%M:%S",
)


def main():
    coordinator = Coordinator()
    server = GameWSServer(coordinator)
    asyncio.run(server.run())


if __name__ == "__main__":
    main()
