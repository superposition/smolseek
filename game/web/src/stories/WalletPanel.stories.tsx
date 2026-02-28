import { useState } from "react";
import type { Meta, StoryObj } from "@storybook/react";
import WalletPanel from "../components/WalletPanel";
import "../App.css";

const meta: Meta<typeof WalletPanel> = {
  title: "Panels/WalletPanel",
  component: WalletPanel,
  decorators: [
    (Story) => (
      <div style={{ width: 320, background: "#12121e" }}>
        <Story />
      </div>
    ),
  ],
};

export default meta;
type Story = StoryObj<typeof WalletPanel>;

/** Live wallet — connect MetaMask, see real balance, join game */
export const ConnectWallet: Story = {
  render: () => {
    const [playerId, setPlayerId] = useState<string | null>(null);
    return (
      <WalletPanel
        playerId={playerId}
        onJoin={(id) => setPlayerId(id)}
      />
    );
  },
};
