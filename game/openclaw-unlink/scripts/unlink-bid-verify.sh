#!/usr/bin/env bash
set -euo pipefail

BASE="${UNLINK_SERVICE_URL:-http://localhost:3001}"

RELAY_ID="" PLAYER_ID="" CACHE_ID="" ROUND="0"

usage() {
  echo "Usage: $0 --relay-id <id> --player-id <id> --cache-id <id> [--round <n>]" >&2
  echo "" >&2
  echo "Verify that a player's private bid landed in escrow." >&2
  echo "May take up to 30 seconds while polling the Unlink relay." >&2
  exit 1
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --relay-id)  RELAY_ID="$2";  shift 2 ;;
    --player-id) PLAYER_ID="$2"; shift 2 ;;
    --cache-id)  CACHE_ID="$2";  shift 2 ;;
    --round)     ROUND="$2";     shift 2 ;;
    *) usage ;;
  esac
done

if [[ -z "$RELAY_ID" || -z "$PLAYER_ID" || -z "$CACHE_ID" ]]; then
  usage
fi

curl -sf --max-time 35 "$BASE/bid" \
  -H "Content-Type: application/json" \
  -d "{\"relayId\":\"$RELAY_ID\",\"playerId\":\"$PLAYER_ID\",\"cacheId\":\"$CACHE_ID\",\"round\":$ROUND}" \
  | jq '.'
