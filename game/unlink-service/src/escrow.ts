import { initUnlink, createSqliteStorage, waitForConfirmation } from "@unlink-xyz/node";
import type { Unlink } from "@unlink-xyz/node";

// Monad testnet MON token address
export const MON_TOKEN = "0xEeeeeEeeeEeEeeEeEeEeeEEEeeeeEeeeeeeeEEeE";

let unlink: Unlink;

export async function getUnlink(): Promise<Unlink> {
  if (unlink) return unlink;

  const mnemonic = process.env.UNLINK_MNEMONIC;

  if (mnemonic) {
    // Import existing mnemonic (laptop with funded account)
    unlink = await initUnlink({
      chain: "monad-testnet",
      storage: createSqliteStorage({ path: "./escrow.db" }),
      setup: false,
      sync: false,
    });
    await unlink.seed.importMnemonic(mnemonic.trim());
    await unlink.accounts.create();
    await unlink.sync();
  } else {
    // Fresh wallet (auto-creates seed + account)
    unlink = await initUnlink({
      chain: "monad-testnet",
      storage: createSqliteStorage({ path: "./escrow.db" }),
    });
  }

  return unlink;
}

export async function getEscrowAddress(): Promise<string> {
  const u = await getUnlink();
  const account = await u.accounts.getActive();
  if (!account) throw new Error("No active escrow account");
  return account.address;
}

export async function verifyBid(relayId: string): Promise<{ confirmed: boolean }> {
  const u = await getUnlink();
  try {
    await waitForConfirmation(u, relayId, {
      timeout: 30_000,
      pollInterval: 2_000,
    });
    return { confirmed: true };
  } catch (err: any) {
    if (err.name === "TimeoutError") {
      return { confirmed: false };
    }
    throw err;
  }
}

export async function distribute(
  recipientAddress: string,
  amount: bigint,
): Promise<{ relayId: string }> {
  const u = await getUnlink();
  const result = await u.send({
    transfers: [
      {
        token: MON_TOKEN,
        recipient: recipientAddress,
        amount,
      },
    ],
  });
  return { relayId: result.relayId };
}

export async function getBalance(): Promise<bigint> {
  const u = await getUnlink();
  const balance = await u.getBalance(MON_TOKEN);
  return balance;
}
