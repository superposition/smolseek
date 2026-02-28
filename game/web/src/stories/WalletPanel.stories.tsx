import type { Meta, StoryObj } from "@storybook/react";
import WalletPanel from "../components/WalletPanel";
import "../App.css";

const meta: Meta<typeof WalletPanel> = {
  title: "Panels/WalletPanel",
  component: WalletPanel,
  decorators: [(Story) => <div style={{ width: 320, background: "#12121e" }}><Story /></div>],
};

export default meta;
type Story = StoryObj<typeof WalletPanel>;

export const NotJoined: Story = {
  args: { playerId: null, onJoin: (id, addr) => alert(`Joined: ${id} @ ${addr}`) },
};

export const JoinedAsAlice: Story = {
  args: { playerId: "alice", onJoin: () => {} },
};

export const JoinedAsLongName: Story = {
  args: { playerId: "xX_DragonSlayer_2026_Xx", onJoin: () => {} },
};
