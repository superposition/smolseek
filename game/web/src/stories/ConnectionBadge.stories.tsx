import type { Meta, StoryObj } from "@storybook/react";
import ConnectionBadge from "../components/ConnectionBadge";
import "../App.css";

const meta: Meta<typeof ConnectionBadge> = {
  title: "Panels/ConnectionBadge",
  component: ConnectionBadge,
  decorators: [(Story) => <div style={{ width: 320, background: "#12121e", padding: 16 }}><Story /></div>],
};

export default meta;
type Story = StoryObj<typeof ConnectionBadge>;

export const AllGreen: Story = {
  args: { websocket: "ok", escrow: "ok" },
};

export const EscrowDown: Story = {
  args: { websocket: "ok", escrow: "err" },
};

export const WebSocketReconnecting: Story = {
  args: { websocket: "warn", escrow: "ok" },
};

export const AllDown: Story = {
  args: { websocket: "err", escrow: "err" },
};
