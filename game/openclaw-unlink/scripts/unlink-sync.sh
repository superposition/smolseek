#!/usr/bin/env bash
set -euo pipefail

BASE="${UNLINK_SERVICE_URL:-http://localhost:3001}"

echo "Syncing escrow wallet with relay..."
curl -sf "$BASE/sync" | jq '.'
