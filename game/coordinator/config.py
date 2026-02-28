"""Game configuration constants."""

# ---- Network ----
GAME_WS_PORT = 8081               # Browser-facing WS server
JETSON_WS_HOST = "192.168.0.221"  # Jetson upstream
JETSON_WS_PORT = 9090             # Jetson point_cloud_ws port
UNLINK_SERVICE_URL = "http://localhost:3001"

# ---- Game rules ----
NUM_ROUNDS = 5
DEPOSIT_DURATION_SEC = 60         # Time for players to fund escrow
BID_DURATION_SEC = 30             # Time to submit bids per round
NAV_TIMEOUT_SEC = 30              # Max time for robot to reach cache
MIN_PLAYERS = 2                   # Minimum to start

# ---- Caches (hardcoded for now — skip B2) ----
# Coordinates in map frame (meters), value in MON wei
DEFAULT_CACHES = [
    {"id": "cache_1", "x": 1.0,  "y": 0.5,  "z": 0.0, "value": "500000000000000000"},
    {"id": "cache_2", "x": -0.5, "y": 1.5,  "z": 0.0, "value": "750000000000000000"},
    {"id": "cache_3", "x": 2.0,  "y": -1.0, "z": 0.0, "value": "1000000000000000000"},
    {"id": "cache_4", "x": -1.0, "y": -0.5, "z": 0.0, "value": "500000000000000000"},
    {"id": "cache_5", "x": 0.5,  "y": 2.0,  "z": 0.0, "value": "750000000000000000"},
]

# ---- WS protocol message types ----
# Existing (from Jetson)
MSG_FULL_CLOUD   = 0x01
MSG_DELTA_POINTS = 0x02
MSG_TRAJECTORY   = 0x03
MSG_STATUS       = 0x04
MSG_NAV_STATUS   = 0x14

# Game layer
MSG_GAME_STATE   = 0x10
MSG_CACHE_LOCS   = 0x11
MSG_ROBOT_TARGET = 0x12
MSG_ROUND_RESULT = 0x13
