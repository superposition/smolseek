#!/usr/bin/env bash
set -euo pipefail

BASE="${UNLINK_SERVICE_URL:-http://localhost:3001}"

curl -sf "$BASE/liquidity/burner" | jq '.'
