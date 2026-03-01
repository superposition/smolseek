#!/usr/bin/env bash
set -euo pipefail

BASE="${UNLINK_SERVICE_URL:-http://localhost:3001}"

TOKEN="" AMOUNT="" TO=""

usage() {
  echo "Usage: $0 --token <address> --amount <wei> [--to <address>]" >&2
  echo "" >&2
  echo "Mint tokens on a deployed mintable ERC20." >&2
  echo "" >&2
  echo "  --token   Token contract address" >&2
  echo "  --amount  Amount to mint in wei" >&2
  echo "  --to      Recipient address (default: burner wallet)" >&2
  exit 1
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --token)  TOKEN="$2";  shift 2 ;;
    --amount) AMOUNT="$2"; shift 2 ;;
    --to)     TO="$2";     shift 2 ;;
    *) usage ;;
  esac
done

if [[ -z "$TOKEN" || -z "$AMOUNT" ]]; then
  usage
fi

BODY="{\"token\":\"$TOKEN\",\"amount\":\"$AMOUNT\""
if [[ -n "$TO" ]]; then
  BODY="$BODY,\"to\":\"$TO\""
fi
BODY="$BODY}"

HTTP_CODE=$(curl -s -o /tmp/unlink-mint-response.json -w '%{http_code}' \
  "$BASE/liquidity/mint" \
  -H "Content-Type: application/json" \
  -d "$BODY" \
  --max-time 120 \
  2>/dev/null) || true

RESPONSE=$(cat /tmp/unlink-mint-response.json 2>/dev/null || echo '{}')

if [[ "$HTTP_CODE" == "200" ]]; then
  echo "$RESPONSE" | jq '.'
else
  echo "Error (HTTP $HTTP_CODE):" >&2
  echo "$RESPONSE" | jq '.' 2>/dev/null || echo "$RESPONSE" >&2
  exit 1
fi
