import { Router } from "express";
import { verifyBid } from "../escrow.js";
import Database from "better-sqlite3";

const router = Router();

// SQLite for bid storage
const db = new Database("./bids.db");
db.exec(`
  CREATE TABLE IF NOT EXISTS bids (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    relay_id TEXT UNIQUE NOT NULL,
    player_id TEXT NOT NULL,
    cache_id TEXT NOT NULL,
    amount TEXT NOT NULL,
    round INTEGER NOT NULL DEFAULT 0,
    confirmed INTEGER NOT NULL DEFAULT 0,
    created_at TEXT NOT NULL DEFAULT (datetime('now'))
  )
`);

const insertBid = db.prepare(
  `INSERT OR IGNORE INTO bids (relay_id, player_id, cache_id, amount, round, confirmed)
   VALUES (?, ?, ?, ?, ?, ?)`
);

const getBidsForRound = db.prepare(
  `SELECT * FROM bids WHERE round = ? AND confirmed = 1 ORDER BY CAST(amount AS INTEGER) DESC`
);

// POST /bid - verify a private bid landed in escrow
router.post("/", async (req, res) => {
  const { relayId, playerId, cacheId, round, amount } = req.body;

  if (!relayId || !playerId || !cacheId) {
    res.status(400).json({ error: "Missing relayId, playerId, or cacheId" });
    return;
  }

  try {
    const { confirmed } = await verifyBid(relayId);
    const bidAmount = amount ?? "0";

    if (confirmed) {
      insertBid.run(relayId, playerId, cacheId, bidAmount.toString(), round ?? 0, 1);
    }

    res.json({
      confirmed,
      amount: bidAmount.toString(),
      relayId,
      bidId: relayId,
    });
  } catch (err: any) {
    console.error("Bid verification error:", err);
    res.status(500).json({ error: err.message });
  }
});

// GET /bids/:round - get all confirmed bids for a round
router.get("/:round", (req, res) => {
  const round = parseInt(req.params.round, 10);
  const bids = getBidsForRound.all(round);
  res.json({ bids });
});

export default router;
