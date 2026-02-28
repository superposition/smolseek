import { initUnlink, createSqliteStorage, waitForConfirmation } from "@unlink-xyz/node";
import type { Unlink } from "@unlink-xyz/node";

// Monad testnet MON token address
export const MON_TOKEN = "0xEeeeeEeeeEeEeeEeEeEeeEEEeeeeEeeeeeeeEEeE";

let unlink: Unlink;

export async function getUnlink(): Promise<Unlink> {
  if (unlink) return unlink;

  unlink = await initUnlink({
    chain: "monad-testnet",
    storage: createSqliteStorage({ path: "./escrow.db" }),
  });

  return unlink;
}

export async function getEscrowAddress(): Promise<string> {
  const u = await getUnlink();
  const account = u.activeAccount;
  if (!account) throw new Error("No active escrow account");
  return account.address;
}

export async function verifyBid(relayId: string): Promise<{ confirmed: boolean; amount: bigint }> {
  const u = await getUnlink();
  try {
    const status = await waitForConfirmation(u, relayId, {
      timeout: 30_000,
      pollInterval: 2_000,
    });
    // Extract the transfer amount from the confirmed status
    return { confirmed: true, amount: BigInt(status.amount ?? 0) };
  } catch (err: any) {
    if (err.name === "TimeoutError") {
      return { confirmed: false, amount: 0n };
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
