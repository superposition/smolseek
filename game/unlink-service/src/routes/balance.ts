import { Router } from "express";
import { getBalance, getEscrowAddress } from "../escrow.js";

const router = Router();

// GET /balance - escrow balance
router.get("/", async (_req, res) => {
  try {
    const balance = await getBalance();
    res.json({ balance: balance.toString() });
  } catch (err: any) {
    console.error("Balance error:", err);
    res.status(500).json({ error: err.message });
  }
});

// GET /escrow-address - get the escrow's Unlink address
router.get("/escrow-address", async (_req, res) => {
  try {
    const address = await getEscrowAddress();
    res.json({ address });
  } catch (err: any) {
    console.error("Escrow address error:", err);
    res.status(500).json({ error: err.message });
  }
});

// GET /game-pool - combined info for the game
router.get("/game-pool", async (_req, res) => {
  try {
    const [address, balance] = await Promise.all([getEscrowAddress(), getBalance()]);
    res.json({ address, balance: balance.toString() });
  } catch (err: any) {
    console.error("Game pool error:", err);
    res.status(500).json({ error: err.message });
  }
});

export default router;
