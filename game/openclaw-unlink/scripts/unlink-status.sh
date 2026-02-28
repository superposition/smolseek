#!/usr/bin/env bash
set -euo pipefail

BASE="${UNLINK_SERVICE_URL:-http://localhost:3001}"

if ! curl -sf "$BASE/health" > /dev/null 2>&1; then
  echo "Error: Unlink service not reachable at $BASE" >&2
  echo "Start it with: cd game/unlink-service && npm run dev" >&2
  exit 1
fi

echo "Unlink Service: OK"

address=$(curl -sf "$BASE/escrow-address" | jq -r '.address')
echo "Escrow Address: $address"

balance=$(curl -sf "$BASE/balance" | jq -r '.balance')
echo "MON Balance:    $balance"
