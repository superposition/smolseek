#!/usr/bin/env bash
# E1: End-to-end integration test runner
# Usage:
#   ./game/scripts/run_integration.sh             # mock mode (no hardware)
#   ./game/scripts/run_integration.sh --live       # real Jetson + Unlink
#   ./game/scripts/run_integration.sh --fallbacks  # test error scenarios

set -euo pipefail
cd "$(dirname "$0")/../.."

echo "========================================"
echo " smolseek — Integration Test"
echo "========================================"

# Ensure ports are free
for port in 8081 9090 3001; do
    if lsof -ti ":$port" >/dev/null 2>&1; then
        echo "ERROR: Port $port is in use. Kill the process first."
        exit 1
    fi
done

# Run the test
python -m game.tests.integration_test "$@"
