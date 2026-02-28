#!/usr/bin/env bash
set -euo pipefail

BASE="${UNLINK_SERVICE_URL:-http://localhost:3001}"

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <round>" >&2
  echo "  List all confirmed bids for a game round (sorted by amount desc)." >&2
  exit 1
fi

ROUND="$1"

curl -sf "$BASE/bid/$ROUND" | jq '.'
