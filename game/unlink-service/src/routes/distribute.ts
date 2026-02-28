import { Router } from "express";
import { distribute } from "../escrow.js";
import Database from "better-sqlite3";

const router = Router();

// Track distributions to prevent double-sends
const db = new Database("./bids.db");
db.exec(`
  CREATE TABLE IF NOT EXISTS distributions (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    round INTEGER NOT NULL,
    recipient TEXT NOT NULL,
    amount TEXT NOT NULL,
    relay_id TEXT NOT NULL,
    created_at TEXT NOT NULL DEFAULT (datetime('now')),
    UNIQUE(round, recipient)
  )
`);

const checkDistributed = db.prepare(
  `SELECT * FROM distributions WHERE round = ? AND recipient = ?`
);
const insertDistribution = db.prepare(
  `INSERT INTO distributions (round, recipient, amount, relay_id) VALUES (?, ?, ?, ?)`
);

// POST /distribute - send winnings from escrow to winner
router.post("/", async (req, res) => {
  const { recipientAddress, amount, round } = req.body;

  if (!recipientAddress || !amount) {
    res.status(400).json({ error: "Missing recipientAddress or amount" });
    return;
  }

  // Prevent double distribution
  const existing = checkDistributed.get(round ?? 0, recipientAddress);
  if (existing) {
    res.status(409).json({ error: "Already distributed for this round", existing });
    return;
  }

  try {
    const result = await distribute(recipientAddress, BigInt(amount));
    insertDistribution.run(round ?? 0, recipientAddress, amount.toString(), result.relayId);
    res.json({ relayId: result.relayId, status: "sent" });
  } catch (err: any) {
    console.error("Distribution error:", err);
    res.status(500).json({ error: err.message });
  }
});

export default router;
