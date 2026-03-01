#!/usr/bin/env bash
set -euo pipefail

BASE="${UNLINK_SERVICE_URL:-http://localhost:3001}"

TOKEN_A="" TOKEN_B="" AMOUNT_A="" AMOUNT_B=""

usage() {
  echo "Usage: $0 --token-a <address> --token-b <address> --amount-a <wei> --amount-b <wei>" >&2
  echo "" >&2
  echo "Add liquidity to a Uniswap V2 pool on Monad testnet." >&2
  echo "Approves both tokens for the router, then calls addLiquidity." >&2
  echo "The burner wallet must hold both tokens and native MON for gas." >&2
  exit 1
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --token-a)  TOKEN_A="$2";  shift 2 ;;
    --token-b)  TOKEN_B="$2";  shift 2 ;;
    --amount-a) AMOUNT_A="$2"; shift 2 ;;
    --amount-b) AMOUNT_B="$2"; shift 2 ;;
    *) usage ;;
  esac
done

if [[ -z "$TOKEN_A" || -z "$TOKEN_B" || -z "$AMOUNT_A" || -z "$AMOUNT_B" ]]; then
  usage
fi

HTTP_CODE=$(curl -s -o /tmp/unlink-add-liq-response.json -w '%{http_code}' \
  "$BASE/liquidity/add" \
  -H "Content-Type: application/json" \
  -d "{\"tokenA\":\"$TOKEN_A\",\"tokenB\":\"$TOKEN_B\",\"amountA\":\"$AMOUNT_A\",\"amountB\":\"$AMOUNT_B\"}" \
  --max-time 120 \
  2>/dev/null) || true

RESPONSE=$(cat /tmp/unlink-add-liq-response.json 2>/dev/null || echo '{}')

if [[ "$HTTP_CODE" == "200" ]]; then
  echo "$RESPONSE" | jq '.'
else
  echo "Error (HTTP $HTTP_CODE):" >&2
  echo "$RESPONSE" | jq '.' 2>/dev/null || echo "$RESPONSE" >&2
  exit 1
fi
