# @unlink-xyz/node SDK Reference

Package: `@unlink-xyz/node` (canary)
Chain: `monad-testnet`
Token: MON (`0xEeeeeEeeeEeEeeEeEeEeeEEEeeeeEeeeeeeeEEeE`)

## Initialization

```typescript
import { initUnlink, createSqliteStorage, waitForConfirmation } from "@unlink-xyz/node";

const unlink = await initUnlink({
  chain: "monad-testnet",
  storage: createSqliteStorage({ path: "./escrow.db" }),
});
```

`initUnlink` creates or restores an Unlink wallet. The SQLite storage persists the account state across restarts.

## Account

```typescript
const address = unlink.activeAccount.address;
```

Returns the escrow wallet's Unlink address. Throws if no active account.

## Balance

```typescript
const balance: bigint = await unlink.getBalance(MON_TOKEN);
```

Returns the shielded MON balance in wei.

## Send (Distribute)

```typescript
const result = await unlink.send({
  transfers: [{
    token: "0xEeeeeEeeeEeEeeEeEeEeeEEEeeeeEeeeeeeeEEeE",
    recipient: "0xRecipientAddress",
    amount: BigInt("1000000000000000000"), // 1 MON
  }],
});

console.log(result.relayId); // relay transaction ID
```

Sends a private transfer via the Unlink relay. The `relayId` can be used to track confirmation.

## Wait for Confirmation (Bid Verification)

```typescript
const status = await waitForConfirmation(unlink, relayId, {
  timeout: 30_000,    // 30 seconds
  pollInterval: 2_000, // poll every 2 seconds
});

console.log(status.amount); // confirmed transfer amount
```

Polls the Unlink relay until the transaction with the given `relayId` is confirmed. Throws `TimeoutError` if not confirmed within the timeout.

## Constants

```typescript
// Monad testnet MON token (native token placeholder address)
const MON_TOKEN = "0xEeeeeEeeeEeEeeEeEeEeeEEEeeeeEeeeeeeeEEeE";
```

## Source Files

- `game/unlink-service/src/escrow.ts` — Core SDK wrapper functions
- `game/unlink-service/src/routes/bid.ts` — Bid verification endpoint
- `game/unlink-service/src/routes/distribute.ts` — Distribution endpoint
- `game/unlink-service/src/routes/balance.ts` — Balance endpoints
