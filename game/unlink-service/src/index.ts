import express from "express";
import cors from "cors";
import { getUnlink, getEscrowAddress } from "./escrow.js";
import bidRouter from "./routes/bid.js";
import distributeRouter from "./routes/distribute.js";
import balanceRouter from "./routes/balance.js";

const app = express();
const PORT = parseInt(process.env.PORT ?? "3001", 10);

app.use(cors());
app.use(express.json());

// Routes
app.use("/bid", bidRouter);
app.use("/distribute", distributeRouter);
app.use("/balance", balanceRouter);

// Health check
app.get("/health", (_req, res) => {
  res.json({ status: "ok", service: "smolseek-unlink" });
});

// Escrow address (convenience top-level route)
app.get("/escrow-address", async (_req, res) => {
  try {
    const address = await getEscrowAddress();
    res.json({ address });
  } catch (err: any) {
    res.status(500).json({ error: err.message });
  }
});

// Start
async function main() {
  console.log("Initializing Unlink escrow wallet...");
  await getUnlink();
  const address = await getEscrowAddress();
  console.log(`Escrow address: ${address}`);

  app.listen(PORT, () => {
    console.log(`smolseek-unlink service running on http://localhost:${PORT}`);
    console.log(`  POST /bid          - verify a private bid`);
    console.log(`  POST /distribute   - send winnings to winner`);
    console.log(`  GET  /balance      - escrow balance`);
    console.log(`  GET  /escrow-address - escrow Unlink address`);
    console.log(`  GET  /health       - health check`);
  });
}

main().catch((err) => {
  console.error("Fatal:", err);
  process.exit(1);
});
