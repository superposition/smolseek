# smolseek game

Privacy-preserving claw machine game on Monad testnet using [Unlink](https://unlink.xyz) for ZK-private payments.

## Architecture

```
game/
  unlink-service/    # Express escrow service (localhost:3001)
  openclaw-unlink/   # OpenClaw skill module — shell scripts + SKILL.md
```

- **unlink-service** — Express server managing a ZK escrow wallet via `@unlink-xyz/node`. Handles bid verification, prize distribution, balance queries, and relay sync.
- **openclaw-unlink** — Agent-friendly shell scripts that call the service REST API. Designed as an OpenClaw skill module so the bot can manage payments autonomously.

## Quick Start

```bash
# 1. Start the escrow service (fresh wallet, no mnemonic)
cd game/unlink-service
npm install
npm run dev

# 2. Verify
curl http://localhost:3001/health
```

## Scripts (openclaw-unlink)

| Script | Description |
|--------|-------------|
| `unlink-status.sh` | Health check + escrow summary |
| `unlink-address.sh` | Get escrow Unlink address |
| `unlink-balance.sh` | Check token balance (`--pool` for combined view) |
| `unlink-sync.sh` | Re-scan relay for incoming transfers |
| `unlink-bid-verify.sh` | Verify a player's private bid by relay ID |
| `unlink-bid-list.sh` | List confirmed bids for a round |
| `unlink-distribute.sh` | Send winnings (with double-send protection) |
| `unlink-pool.sh` | Combined address + balance JSON |

## End-to-End Test (verified 2025-02-28)

Full cycle tested laptop-to-Jetson-bot:

1. Start service on bot with fresh wallet (auto-creates seed)
2. Send tokens from laptop (`unlink-cli send`) to bot's escrow address
3. Sync bot (`unlink-sync.sh`) to detect incoming transfer
4. Verify bid on bot (`unlink-bid-verify.sh` with relay ID)
5. Distribute winnings from bot back to laptop (`unlink-distribute.sh`)
6. Double-distribute blocked with 409

All 8 scripts pass. Write operations (bid-verify, distribute) confirmed on Monad testnet relay.

## Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `UNLINK_SERVICE_URL` | `http://localhost:3001` | Base URL for scripts |
| `UNLINK_MNEMONIC` | *(none)* | Import existing wallet; omit for fresh |
| `ESCROW_TOKEN` | `0xaaa4...df6b` | Token contract address |
| `PORT` | `3001` | Service listen port |
