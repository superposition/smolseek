import { Router } from "express";
import {
  deployToken,
  mintToken,
  addLiquidity,
  getBurnerAddress,
  getBurnerBalance,
} from "../escrow.js";

const router = Router();

// GET /liquidity/burner — burner wallet address + native balance
router.get("/burner", async (_req, res) => {
  try {
    const [address, balance] = await Promise.all([
      getBurnerAddress(),
      getBurnerBalance(),
    ]);
    res.json({ address, nativeBalance: balance.toString() });
  } catch (err: any) {
    console.error("Burner info error:", err);
    res.status(500).json({ error: err.message });
  }
});

// POST /liquidity/deploy — deploy a new mintable ERC20
router.post("/deploy", async (req, res) => {
  try {
    const { name, symbol, initialSupply } = req.body;
    if (!name || !symbol) {
      res.status(400).json({ error: "Missing required fields: name, symbol" });
      return;
    }
    const supply = BigInt(initialSupply ?? "0");
    const result = await deployToken(name, symbol, supply);
    const burner = await getBurnerAddress();
    res.json({ ...result, burnerAddress: burner });
  } catch (err: any) {
    console.error("Deploy error:", err);
    res.status(500).json({ error: err.message });
  }
});

// POST /liquidity/mint — mint tokens on a deployed ERC20
router.post("/mint", async (req, res) => {
  try {
    const { token, to, amount } = req.body;
    if (!token || !amount) {
      res.status(400).json({ error: "Missing required fields: token, amount" });
      return;
    }
    // Default mint target is the burner wallet itself
    const mintTo = to || (await getBurnerAddress());
    const result = await mintToken(token, mintTo, BigInt(amount));
    res.json({ ...result, to: mintTo });
  } catch (err: any) {
    console.error("Mint error:", err);
    res.status(500).json({ error: err.message });
  }
});

// POST /liquidity/add — add liquidity to Uniswap V2 pool
router.post("/add", async (req, res) => {
  try {
    const { tokenA, tokenB, amountA, amountB } = req.body;
    if (!tokenA || !tokenB || !amountA || !amountB) {
      res.status(400).json({
        error: "Missing required fields: tokenA, tokenB, amountA, amountB",
      });
      return;
    }
    const result = await addLiquidity(
      tokenA,
      tokenB,
      BigInt(amountA),
      BigInt(amountB),
    );
    res.json(result);
  } catch (err: any) {
    console.error("Add liquidity error:", err);
    res.status(500).json({ error: err.message });
  }
});

export default router;
