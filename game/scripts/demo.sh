#!/usr/bin/env bash
# ============================================================
# smolseek — 3-Minute Judge Demo Runner
# ============================================================
#
# Usage:
#   ./game/scripts/demo.sh              # full demo (real Jetson + Unlink)
#   ./game/scripts/demo.sh --mock       # mock mode (no hardware)
#   ./game/scripts/demo.sh --prefund    # pre-fund judge wallets only
#   ./game/scripts/demo.sh --spectator  # launch spectator view only
#
# Prerequisites:
#   1. Jetson is up:  ssh jetson@192.168.0.221
#   2. ROS2 stack:    autonomous_mapping.launch.py running
#   3. Unlink service: cd game/unlink-service && npm run dev
#   4. Web app built:  cd game/web && npm run build
#
# Demo Script (3 minutes):
#   0:00  "The robot mapped this room."         — spectator shows 3D point cloud
#   0:30  "It hid tokens at locations it found." — host starts game, caches appear
#   0:45  "Now you play."                        — judges scan QR, connect wallets, bid
#   1:45  "Robot dispatched."                    — robot drives to winning cache
#   2:30  "Nobody knows who won."               — show Monad explorer, ZK proofs
#   3:00  Done.
# ============================================================

set -euo pipefail
cd "$(dirname "$0")/../.."

UNLINK_URL="${UNLINK_SERVICE_URL:-http://localhost:3001}"
GAME_WS_PORT=8081
WEB_PORT=5173
MOCK_MODE=false
ACTION="demo"

# Parse args
for arg in "$@"; do
    case "$arg" in
        --mock)     MOCK_MODE=true ;;
        --prefund)  ACTION="prefund" ;;
        --spectator) ACTION="spectator" ;;
    esac
done

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

log()  { echo -e "${GREEN}[demo]${NC} $*"; }
warn() { echo -e "${YELLOW}[demo]${NC} $*"; }
err()  { echo -e "${RED}[demo]${NC} $*"; }
mark() { echo -e "${CYAN}${BOLD}[T+$1]${NC} $2"; }

cleanup() {
    log "Cleaning up..."
    # Kill background processes
    [ -n "${COORD_PID:-}" ] && kill "$COORD_PID" 2>/dev/null || true
    [ -n "${WEB_PID:-}" ] && kill "$WEB_PID" 2>/dev/null || true
    [ -n "${MOCK_PID:-}" ] && kill "$MOCK_PID" 2>/dev/null || true
}
trap cleanup EXIT

# ============================================================
# Pre-fund judge wallets
# ============================================================
prefund_wallets() {
    log "Pre-funding judge wallets..."

    # Check Unlink service is up
    if ! curl -sf "$UNLINK_URL/health" >/dev/null 2>&1; then
        err "Unlink service not running at $UNLINK_URL"
        err "Start it: cd game/unlink-service && npm run dev"
        exit 1
    fi

    ESCROW=$(curl -sf "$UNLINK_URL/escrow-address" | python3 -c "import sys,json; print(json.load(sys.stdin)['address'])")
    BALANCE=$(curl -sf "$UNLINK_URL/balance" | python3 -c "import sys,json; print(json.load(sys.stdin)['balance'])")
    log "Escrow address: $ESCROW"
    log "Escrow balance: $BALANCE wei"

    echo ""
    echo -e "${BOLD}Judge Wallet Instructions:${NC}"
    echo "1. Judges open the game URL on their phones"
    echo "2. They enter a name and join the game"
    echo "3. In production: they would connect their Monad wallet and deposit to:"
    echo -e "   ${CYAN}$ESCROW${NC}"
    echo ""
    echo "For demo: wallets are mocked — no real deposit needed."
}

# ============================================================
# Launch spectator view
# ============================================================
launch_spectator() {
    log "Launching spectator view..."

    # Check if web dev server is running
    if curl -sf "http://localhost:$WEB_PORT" >/dev/null 2>&1; then
        log "Web dev server already running on :$WEB_PORT"
    else
        log "Starting web dev server..."
        cd game/web
        npx vite --port "$WEB_PORT" &
        WEB_PID=$!
        cd ../..
        sleep 3
    fi

    SPECTATOR_URL="http://localhost:$WEB_PORT/?mode=spectator"
    log "Opening spectator view: $SPECTATOR_URL"
    open "$SPECTATOR_URL" 2>/dev/null || xdg-open "$SPECTATOR_URL" 2>/dev/null || true
    echo ""
    echo -e "${BOLD}Spectator URL:${NC} $SPECTATOR_URL"
    echo "Display this on the projector / external monitor."
}

# ============================================================
# Check prerequisites
# ============================================================
check_prereqs() {
    local ok=true

    if [ "$MOCK_MODE" = true ]; then
        log "Mock mode — skipping hardware checks"
        return 0
    fi

    # Check Jetson SSH
    if ssh -o ConnectTimeout=3 jetson@192.168.0.221 "echo ok" >/dev/null 2>&1; then
        log "Jetson SSH: OK"
    else
        err "Jetson SSH: FAILED (192.168.0.221)"
        ok=false
    fi

    # Check Jetson ROS2
    if ssh -o ConnectTimeout=3 jetson@192.168.0.221 \
        "docker exec ugv_jetson_ros_humble bash -c 'source /opt/ros/humble/setup.bash && ros2 node list 2>/dev/null'" \
        2>/dev/null | grep -q "point_cloud_ws"; then
        log "Jetson ROS2 stack: OK"
    else
        warn "Jetson ROS2 stack: NOT RUNNING"
        warn "Launch it: ssh jetson@192.168.0.221"
        warn "  docker exec -it ugv_jetson_ros_humble bash"
        warn "  ros2 launch ugv_map_bridge autonomous_mapping.launch.py"
        ok=false
    fi

    # Check Unlink service
    if curl -sf "$UNLINK_URL/health" >/dev/null 2>&1; then
        log "Unlink service: OK"
    else
        warn "Unlink service: NOT RUNNING"
        warn "Start it: cd game/unlink-service && npm run dev"
        ok=false
    fi

    if [ "$ok" = false ]; then
        err "Some prerequisites not met. Fix above issues or use --mock"
        exit 1
    fi
}

# ============================================================
# Main demo flow
# ============================================================
run_demo() {
    echo ""
    echo -e "${BOLD}========================================${NC}"
    echo -e "${BOLD}   smolseek — Judge Demo${NC}"
    echo -e "${BOLD}========================================${NC}"
    echo ""

    check_prereqs

    # Start coordinator
    if [ "$MOCK_MODE" = true ]; then
        log "Starting coordinator in mock mode..."
        python -m game.tests.integration_test --live 2>&1 &
        MOCK_PID=$!
        # In mock mode, the integration test runs the whole flow automatically
        wait "$MOCK_PID" || true
        return
    fi

    log "Starting game coordinator..."
    python -m game.coordinator &
    COORD_PID=$!
    sleep 2

    # Launch spectator
    launch_spectator

    # Get local IP for QR code
    LOCAL_IP=$(ipconfig getifaddr en0 2>/dev/null || hostname -I 2>/dev/null | awk '{print $1}')
    PLAYER_URL="http://$LOCAL_IP:$WEB_PORT"

    echo ""
    echo -e "${BOLD}========================================${NC}"
    echo -e "${BOLD}   DEMO READY${NC}"
    echo -e "${BOLD}========================================${NC}"
    echo ""
    echo -e "Spectator (projector):  http://localhost:$WEB_PORT/?mode=spectator"
    echo -e "Player (phone):         $PLAYER_URL"
    echo ""
    echo -e "${BOLD}Demo Script:${NC}"
    echo ""
    mark "0:00" "\"The robot mapped this room.\""
    echo "       → Point at projector showing 3D point cloud building"
    echo ""
    mark "0:30" "\"It hid tokens at locations it found.\""
    echo "       → Press START in a player window → caches appear as spheres"
    echo ""
    mark "0:45" "\"Now you play.\""
    echo "       → Hand judges phones, they open $PLAYER_URL"
    echo "       → They enter a name, join, click a cache, submit bid"
    echo ""
    mark "1:45" "\"Robot dispatched.\""
    echo "       → Highest bid wins → robot physically drives to cache"
    echo "       → Crowd watches robot on spectator view"
    echo ""
    mark "2:30" "\"Nobody knows who won.\""
    echo "       → Show Monad block explorer: txns visible but amounts hidden"
    echo "       → Unlink ZK proofs make sender/amount/recipient private"
    echo ""
    mark "3:00" "Done."
    echo ""
    echo -e "${YELLOW}Press Enter when ready to begin, Ctrl+C to abort...${NC}"
    read -r

    log "Demo started! Good luck."
    echo ""

    # Keep running until Ctrl+C
    wait "$COORD_PID" 2>/dev/null || true
}

# ============================================================
# Dispatch
# ============================================================
case "$ACTION" in
    prefund)    prefund_wallets ;;
    spectator)  launch_spectator; echo "Press Ctrl+C to stop."; wait ;;
    demo)       run_demo ;;
esac
