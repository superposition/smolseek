# Unlink Service — API Endpoints

Base URL: `http://localhost:3001` (configurable via `PORT` env var)

## Endpoints

### GET /health

Health check.

**Response:**
```json
{"status": "ok", "service": "smolseek-unlink"}
```

### GET /escrow-address

Get the escrow wallet's Unlink address.

**Response:**
```json
{"address": "0x..."}
```

**Errors:** `500` if no active escrow account.

### POST /bid

Verify that a player's private bid landed in escrow. Polls the Unlink relay for up to 30 seconds.

**Request body:**
```json
{
  "relayId": "0x...",
  "playerId": "player_1",
  "cacheId": "cache_2",
  "round": 1
}
```

| Field | Type | Required | Default |
|-------|------|----------|---------|
| relayId | string | yes | — |
| playerId | string | yes | — |
| cacheId | string | yes | — |
| round | number | no | 0 |

**Response (200):**
```json
{
  "confirmed": true,
  "amount": "1000000000000000",
  "relayId": "0x...",
  "bidId": "0x..."
}
```

**Errors:**
- `400` — Missing `relayId`, `playerId`, or `cacheId`
- `500` — Verification error (non-timeout)

**Notes:**
- If the relay confirmation times out (30s), `confirmed` is `false` and `amount` is `"0"`.
- Confirmed bids are stored in `bids.db` with `confirmed = 1`.
- Duplicate `relayId` values are silently ignored (`INSERT OR IGNORE`).

### GET /bid/:round

Get all confirmed bids for a round, sorted by amount descending.

**URL params:** `round` (integer)

**Response:**
```json
{
  "bids": [
    {
      "id": 1,
      "relay_id": "0x...",
      "player_id": "player_1",
      "cache_id": "cache_2",
      "amount": "1000000000000000",
      "round": 1,
      "confirmed": 1,
      "created_at": "2026-02-28 12:00:00"
    }
  ]
}
```

### POST /distribute

Send MON winnings from escrow to a recipient. Protected against double distribution per round+recipient.

**Request body:**
```json
{
  "recipientAddress": "0x...",
  "amount": "1000000000000000000",
  "round": 1
}
```

| Field | Type | Required | Default |
|-------|------|----------|---------|
| recipientAddress | string | yes | — |
| amount | string | yes | — |
| round | number | no | 0 |

**Response (200):**
```json
{"relayId": "0x...", "status": "sent"}
```

**Errors:**
- `400` — Missing `recipientAddress` or `amount`
- `409` — Already distributed for this round+recipient: `{"error": "Already distributed for this round", "existing": {...}}`
- `500` — Distribution error

### GET /balance

Get the escrow's MON token balance.

**Response:**
```json
{"balance": "1500000000000000000"}
```

### GET /balance/escrow-address

Get the escrow wallet address (same as `/escrow-address`).

**Response:**
```json
{"address": "0x..."}
```

### GET /balance/game-pool

Combined escrow address and balance.

**Response:**
```json
{
  "address": "0x...",
  "balance": "1500000000000000000"
}
```

## Common Error Format

All error responses use:
```json
{"error": "Error message"}
```

## Configuration

| Env var | Default | Description |
|---------|---------|-------------|
| PORT | 3001 | Service listen port |

## Data Storage

- `escrow.db` — Unlink SDK account persistence (SQLite)
- `bids.db` — Bid records and distribution records (SQLite)
