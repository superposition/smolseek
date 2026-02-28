import type { Meta, StoryObj } from "@storybook/react";
import TransactionStatus from "../components/TransactionStatus";
import type { Transaction } from "../components/TransactionStatus";
import "../App.css";

const meta: Meta<typeof TransactionStatus> = {
  title: "Panels/TransactionStatus",
  component: TransactionStatus,
  decorators: [(Story) => <div style={{ width: 320, background: "#12121e" }}><Story /></div>],
};

export default meta;
type Story = StoryObj<typeof TransactionStatus>;

const now = Date.now();

const provingTx: Transaction = {
  relayId: "a1b2c3d4-e5f6-7890-abcd-ef1234567890",
  phase: "proving",
  label: "Bid on cache_3",
  timestamp: now - 3000,
};

const relayingTx: Transaction = {
  relayId: "b2c3d4e5-f6a7-8901-bcde-f12345678901",
  phase: "relaying",
  label: "Bid on cache_1",
  timestamp: now - 8000,
};

const confirmingTx: Transaction = {
  relayId: "c3d4e5f6-a7b8-9012-cdef-123456789012",
  phase: "confirming",
  label: "Distribute to alice",
  timestamp: now - 15000,
};

const confirmedTx: Transaction = {
  relayId: "d4e5f6a7-b8c9-0123-defa-234567890123",
  phase: "confirmed",
  label: "Bid on cache_2",
  timestamp: now - 30000,
};

const failedTx: Transaction = {
  relayId: "e5f6a7b8-c9d0-1234-efab-345678901234",
  phase: "failed",
  label: "Distribute to bob",
  timestamp: now - 20000,
  error: "Insufficient escrow balance",
};

export const SingleProving: Story = {
  args: { transactions: [provingTx] },
};

export const SingleConfirmed: Story = {
  args: { transactions: [confirmedTx] },
};

export const SingleFailed: Story = {
  args: { transactions: [failedTx] },
};

export const MixedPipeline: Story = {
  args: {
    transactions: [confirmedTx, confirmingTx, relayingTx, provingTx],
  },
};

export const WithFailure: Story = {
  args: {
    transactions: [confirmedTx, failedTx, provingTx],
  },
};

export const Empty: Story = {
  args: { transactions: [] },
};
