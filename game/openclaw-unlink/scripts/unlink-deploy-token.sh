#!/usr/bin/env bash
set -euo pipefail

BASE="${UNLINK_SERVICE_URL:-http://localhost:3001}"

NAME="" SYMBOL="" SUPPLY="0"

usage() {
  echo "Usage: $0 --name <name> --symbol <symbol> [--supply <wei>]" >&2
  echo "" >&2
  echo "Deploy a new mintable ERC20 token on Monad testnet." >&2
  echo "The burner wallet must have native MON for gas." >&2
  echo "" >&2
  echo "  --name    Token name (e.g. \"Test Token\")" >&2
  echo "  --symbol  Token symbol (e.g. \"TEST\")" >&2
  echo "  --supply  Initial supply in wei minted to deployer (default: 0)" >&2
  exit 1
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --name)   NAME="$2";   shift 2 ;;
    --symbol) SYMBOL="$2"; shift 2 ;;
    --supply) SUPPLY="$2"; shift 2 ;;
    *) usage ;;
  esac
done

if [[ -z "$NAME" || -z "$SYMBOL" ]]; then
  usage
fi

HTTP_CODE=$(curl -s -o /tmp/unlink-deploy-response.json -w '%{http_code}' \
  "$BASE/liquidity/deploy" \
  -H "Content-Type: application/json" \
  -d "{\"name\":\"$NAME\",\"symbol\":\"$SYMBOL\",\"initialSupply\":\"$SUPPLY\"}" \
  --max-time 120 \
  2>/dev/null) || true

RESPONSE=$(cat /tmp/unlink-deploy-response.json 2>/dev/null || echo '{}')

if [[ "$HTTP_CODE" == "200" ]]; then
  echo "$RESPONSE" | jq '.'
else
  echo "Error (HTTP $HTTP_CODE):" >&2
  echo "$RESPONSE" | jq '.' 2>/dev/null || echo "$RESPONSE" >&2
  exit 1
fi
