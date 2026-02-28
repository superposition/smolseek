#!/usr/bin/env bash
set -euo pipefail

BASE="${UNLINK_SERVICE_URL:-http://localhost:3001}"

RECIPIENT="" AMOUNT="" ROUND="0"

usage() {
  echo "Usage: $0 --recipient <address> --amount <wei> [--round <n>]" >&2
  echo "" >&2
  echo "Send MON winnings from escrow to a recipient." >&2
  echo "Double-send protection: fails with 409 if already distributed for this round+recipient." >&2
  exit 1
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --recipient) RECIPIENT="$2"; shift 2 ;;
    --amount)    AMOUNT="$2";    shift 2 ;;
    --round)     ROUND="$2";     shift 2 ;;
    *) usage ;;
  esac
done

if [[ -z "$RECIPIENT" || -z "$AMOUNT" ]]; then
  usage
fi

HTTP_CODE=$(curl -s -o /tmp/unlink-distribute-response.json -w '%{http_code}' \
  "$BASE/distribute" \
  -H "Content-Type: application/json" \
  -d "{\"recipientAddress\":\"$RECIPIENT\",\"amount\":\"$AMOUNT\",\"round\":$ROUND}" \
  2>/dev/null) || true

RESPONSE=$(cat /tmp/unlink-distribute-response.json 2>/dev/null || echo '{}')

if [[ "$HTTP_CODE" == "409" ]]; then
  echo "Already distributed for round $ROUND to $RECIPIENT" >&2
  echo "$RESPONSE" | jq '.' 2>/dev/null || echo "$RESPONSE"
  exit 1
elif [[ "$HTTP_CODE" == "200" ]]; then
  echo "$RESPONSE" | jq '.'
else
  echo "Error (HTTP $HTTP_CODE):" >&2
  echo "$RESPONSE" | jq '.' 2>/dev/null || echo "$RESPONSE" >&2
  exit 1
fi
