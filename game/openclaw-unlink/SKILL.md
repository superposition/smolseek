---
name: unlink
description: >
  Privacy-preserving payment and DeFi operations for the smolseek game on Monad testnet.
  Use this skill to interact with the Unlink escrow service: check escrow health
  and balance, verify private bids from players, list confirmed bids per round,
  distribute winnings to winners, and execute private token swaps via Uniswap.
  All transactions use ZK proofs via the Unlink protocol — sender, amount, and
  recipient are hidden on-chain. Requires the unlink-service to be running
  (default localhost:3001).
metadata:
  clawdbot:
    emoji: "\U0001F510"
    homepage: "https://unlink.xyz"
    requires:
      bins:
        - curl
        - jq
---

# Unlink

This skill wraps the smolseek Unlink escrow service (`game/unlink-service/`) as agent-friendly shell scripts. The service manages privacy-preserving payments on Monad testnet using the `@unlink-xyz/node` SDK.

## Supported Assets

| Asset | Decimals | Description |
|-------|----------|-------------|
| MON   | 18       | Native Monad testnet token |
| ERC20 | 18       | Any Unlink-supported ERC20 (set `ESCROW_TOKEN`) |

## What it does

- **Health check**: verify the service is running and display escrow summary
- **Address lookup**: get the escrow wallet's Unlink address
- **Balance queries**: check token balance, with optional game-pool combined view
- **Sync**: re-scan the relay for incoming transfers (needed after external sends)
- **Bid verification**: confirm that a player's private bid landed in escrow (ZK proof verification)
- **Bid listing**: list all confirmed bids for a game round, sorted by amount
- **Prize distribution**: send winnings from escrow to a winner's address (with double-send protection)
- **Pool info**: get combined escrow address + balance in one call
- **Token listing**: list available tokens for swapping
- **Private swaps**: execute atomic unshield → swap → reshield via Uniswap V2 (no one sees what was swapped)

## Setup

### 1) Start the Unlink service

```bash
cd game/unlink-service
npm install
npm run dev
```

The service runs on `http://localhost:3001` by default. Verify with:

```bash
curl http://localhost:3001/health
# → {"status":"ok","service":"smolseek-unlink"}
```

### 2) Configure service URL (optional)

If the service runs on a different host/port, set:

```bash
export UNLINK_SERVICE_URL=http://your-host:3001
```

Default: `http://localhost:3001`

### 3) Make scripts executable

```bash
chmod +x skills/unlink/scripts/*.sh
```

## Tools

### unlink-status.sh — Service health + escrow summary

```bash
skills/unlink/scripts/unlink-status.sh
```

Checks service health, displays escrow address and MON balance. Use this as the first call to verify everything is working.

### unlink-address.sh — Get escrow address

```bash
skills/unlink/scripts/unlink-address.sh
```

Returns the raw Unlink escrow wallet address. Use when telling a player where to send funds.

### unlink-balance.sh — Check MON balance

```bash
# Simple balance (wei)
skills/unlink/scripts/unlink-balance.sh

# Combined pool info (address + balance)
skills/unlink/scripts/unlink-balance.sh --pool
```

### unlink-bid-verify.sh — Verify a private bid

```bash
skills/unlink/scripts/unlink-bid-verify.sh \
  --relay-id 0xabc123... \
  --player-id player_1 \
  --cache-id cache_2 \
  --round 1
```

Verifies that a player's private bid actually landed in escrow. This may take up to 30 seconds as the service polls the Unlink relay for confirmation.

**Required args:** `--relay-id`, `--player-id`, `--cache-id`
**Optional args:** `--round` (defaults to 0)

### unlink-bid-list.sh — List bids for a round

```bash
skills/unlink/scripts/unlink-bid-list.sh <round>
```

Returns all confirmed bids for the given round, sorted by amount descending (highest bidder first).

### unlink-distribute.sh — Send winnings

```bash
skills/unlink/scripts/unlink-distribute.sh \
  --recipient 0xdef456... \
  --amount 1000000000000000000 \
  --round 1
```

Sends MON tokens from escrow to the winner. Includes double-send protection — if a distribution has already been made for this round+recipient, it returns an error instead of sending again.

**Required args:** `--recipient`, `--amount` (in wei)
**Optional args:** `--round` (defaults to 0)

### unlink-sync.sh — Sync incoming transfers

```bash
skills/unlink/scripts/unlink-sync.sh
```

Re-scans the Unlink relay for incoming transfers. Call this after sending tokens to the escrow from an external wallet (e.g. `unlink-cli send`) to ensure the balance is up to date.

### unlink-pool.sh — Game pool info

```bash
skills/unlink/scripts/unlink-pool.sh
```

Returns combined escrow address and balance as JSON. Convenience wrapper for agents needing the full pool state.

### unlink-tokens.sh — List available tokens

```bash
skills/unlink/scripts/unlink-tokens.sh
```

Lists all tokens available for swapping on Monad testnet, including their address, symbol, and decimals.

### unlink-swap.sh — Execute a private token swap

```bash
skills/unlink/scripts/unlink-swap.sh \
  --token-in 0xaaa4e95d4da878baf8e10745fdf26e196918df6b \
  --token-out 0x3bd359C1119dA7Da1D913D1C4D2B7c461115433A \
  --amount-in 1000000000000000000 \
  --min-out 0
```

Executes a private token swap via Uniswap V2 on Monad testnet. Tokens unshield from the privacy pool, swap atomically through Uniswap, and output tokens reshield back into the pool. No one sees what was swapped.

**Required args:** `--token-in`, `--token-out`, `--amount-in` (in wei)
**Optional args:** `--min-out` (minimum output amount in wei, defaults to 0)

**Note:** The swap timeout is 120 seconds. Swap will only succeed if there is liquidity in the Uniswap pool for the token pair. Use WMON pairs for best liquidity.

## References

- [API Endpoints](references/api-endpoints.md) — REST endpoint reference with request/response schemas
- [Unlink SDK](references/unlink-sdk.md) — `@unlink-xyz/node` SDK reference

## Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `UNLINK_SERVICE_URL` | `http://localhost:3001` | Base URL of the escrow service |
| `UNLINK_MNEMONIC` | *(none)* | Import existing wallet (omit for fresh wallet) |
| `ESCROW_TOKEN` | `0xaaa4...df6b` | Token address for escrow operations |

## Notes

- All amounts are in **wei** (18 decimals). 1 MON = 1000000000000000000 wei.
- The escrow wallet is persisted in `escrow.db` (SQLite) alongside the service.
- Bid and distribution records are stored in `bids.db` (SQLite).
- After sending tokens externally, call `unlink-sync.sh` before checking balance.
- For privacy safety: never commit `.env` files or database files to git.
