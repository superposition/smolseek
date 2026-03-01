import { Router } from "express";
import { swap, WMON, MON_TOKEN } from "../escrow.js";

const router = Router();

// Known tokens on Monad testnet
const KNOWN_TOKENS = [
  { address: WMON, symbol: "WMON", decimals: 18 },
  { address: MON_TOKEN, symbol: "MON", decimals: 18 },
];

// POST /swap — execute a private swap via Uniswap V2
router.post("/", async (req, res) => {
  try {
    const { tokenIn, tokenOut, amountIn, minAmountOut } = req.body;

    if (!tokenIn || !tokenOut || !amountIn) {
      res.status(400).json({
        error: "Missing required fields: tokenIn, tokenOut, amountIn",
      });
      return;
    }

    const result = await swap(
      tokenIn,
      tokenOut,
      BigInt(amountIn),
      BigInt(minAmountOut ?? "0"),
    );

    res.json({ ...result, status: "sent" });
  } catch (err: any) {
    console.error("Swap error:", err);
    res.status(500).json({ error: err.message });
  }
});

// GET /swap/tokens — list available tokens
router.get("/tokens", async (_req, res) => {
  try {
    res.json({ tokens: KNOWN_TOKENS });
  } catch (err: any) {
    console.error("Token list error:", err);
    res.status(500).json({ error: err.message });
  }
});

export default router;
