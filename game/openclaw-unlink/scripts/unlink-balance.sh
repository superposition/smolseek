#!/usr/bin/env bash
set -euo pipefail

BASE="${UNLINK_SERVICE_URL:-http://localhost:3001}"

POOL=false
for arg in "$@"; do
  case "$arg" in
    --pool) POOL=true ;;
    *)
      echo "Usage: $0 [--pool]" >&2
      echo "  --pool  Show combined address + balance" >&2
      exit 1
      ;;
  esac
done

if [ "$POOL" = true ]; then
  curl -sf "$BASE/balance/game-pool" | jq '.'
else
  curl -sf "$BASE/balance" | jq -r '.balance'
fi
