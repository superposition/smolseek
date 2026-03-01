#!/usr/bin/env bash
set -euo pipefail

BASE="${UNLINK_SERVICE_URL:-http://localhost:3001}"

TOKEN_IN="" TOKEN_OUT="" AMOUNT_IN="" MIN_OUT="0"

usage() {
  echo "Usage: $0 --token-in <address> --token-out <address> --amount-in <wei> [--min-out <wei>]" >&2
  echo "" >&2
  echo "Execute a private token swap via Uniswap V2 on Monad testnet." >&2
  echo "Tokens unshield → swap → reshield atomically. No one sees what was swapped." >&2
  echo "" >&2
  echo "  --token-in   Address of the input token" >&2
  echo "  --token-out  Address of the output token" >&2
  echo "  --amount-in  Amount of input token in wei" >&2
  echo "  --min-out    Minimum output amount in wei (default: 0)" >&2
  exit 1
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --token-in)  TOKEN_IN="$2";  shift 2 ;;
    --token-out) TOKEN_OUT="$2"; shift 2 ;;
    --amount-in) AMOUNT_IN="$2"; shift 2 ;;
    --min-out)   MIN_OUT="$2";   shift 2 ;;
    *) usage ;;
  esac
done

if [[ -z "$TOKEN_IN" || -z "$TOKEN_OUT" || -z "$AMOUNT_IN" ]]; then
  usage
fi

HTTP_CODE=$(curl -s -o /tmp/unlink-swap-response.json -w '%{http_code}' \
  "$BASE/swap" \
  -H "Content-Type: application/json" \
  -d "{\"tokenIn\":\"$TOKEN_IN\",\"tokenOut\":\"$TOKEN_OUT\",\"amountIn\":\"$AMOUNT_IN\",\"minAmountOut\":\"$MIN_OUT\"}" \
  --max-time 120 \
  2>/dev/null) || true

RESPONSE=$(cat /tmp/unlink-swap-response.json 2>/dev/null || echo '{}')

if [[ "$HTTP_CODE" == "200" ]]; then
  echo "$RESPONSE" | jq '.'
else
  echo "Error (HTTP $HTTP_CODE):" >&2
  echo "$RESPONSE" | jq '.' 2>/dev/null || echo "$RESPONSE" >&2
  exit 1
fi
