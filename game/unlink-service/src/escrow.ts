import { initUnlink, createSqliteStorage, waitForConfirmation } from "@unlink-xyz/node";
import type { Unlink } from "@unlink-xyz/node";
import { approve, contract } from "@unlink-xyz/core";

// Monad testnet token address (ERC20 available via faucet)
export const MON_TOKEN = process.env.ESCROW_TOKEN ?? "0xaaa4e95d4da878baf8e10745fdf26e196918df6b";

// Uniswap V2 on Monad testnet
export const UNISWAP_V2_ROUTER = "0x4b2ab38dbf28d31d467aa8993f6c2585981d6804";
export const WMON = "0x3bd359C1119dA7Da1D913D1C4D2B7c461115433A";

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
    await unlink.seed.importMnemonic(mnemonic.trim(), { overwrite: true });
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

export async function syncUnlink(): Promise<void> {
  const u = await getUnlink();
  await u.sync();
}

export async function getBalance(): Promise<bigint> {
  const u = await getUnlink();
  const balance = await u.getBalance(MON_TOKEN);
  return balance;
}

export async function swap(
  tokenIn: string,
  tokenOut: string,
  amountIn: bigint,
  minAmountOut: bigint,
): Promise<{ relayId: string }> {
  const u = await getUnlink();

  // Build approve call — let the V2 router spend the unshielded tokens
  const approveCall = approve(tokenIn, UNISWAP_V2_ROUTER, amountIn);

  // Build swap call via Uniswap V2 Router
  const router = contract(UNISWAP_V2_ROUTER, [
    "function swapExactTokensForTokens(uint256 amountIn, uint256 amountOutMin, address[] path, address to, uint256 deadline) returns (uint256[])",
  ]);

  const deadline = BigInt(Math.floor(Date.now() / 1000) + 600);
  const swapCall = router.swapExactTokensForTokens(
    amountIn,
    minAmountOut,
    [tokenIn, tokenOut],
    u.adapter.address, // output goes to adapter for reshielding
    deadline,
  );

  // Execute: unshield tokenIn → approve → swap → reshield tokenOut
  const result = await u.interact({
    spend: [{ token: tokenIn, amount: amountIn }],
    calls: [approveCall, swapCall],
    receive: [{ token: tokenOut, minAmount: minAmountOut }],
  });

  return { relayId: result.relayId };
}
