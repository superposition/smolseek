import type { Meta, StoryObj } from "@storybook/react";
import EscrowPool from "../components/EscrowPool";
import "../App.css";

const meta: Meta<typeof EscrowPool> = {
  title: "Panels/EscrowPool",
  component: EscrowPool,
  decorators: [(Story) => <div style={{ width: 320, background: "#12121e" }}><Story /></div>],
};

export default meta;
type Story = StoryObj<typeof EscrowPool>;

export const Funded: Story = {
  args: {
    address: "unlink1qyz0qu97u8x6mwtag7tylwtkuhxqyyvchsvndaxggv27pnu6du4gpz53jwtfr9y2jy",
    balance: "3500000000000000000",
  },
};

export const Empty: Story = {
  args: {
    address: "unlink1qy0henu4tc9ph8mg6t04f8thxpu3cupagvaaqvq4p6e465rgttpv9z53jwtfr9y2jx",
    balance: "0",
  },
};

export const SmallBalance: Story = {
  args: {
    address: "unlink1qyz0qu97u8x6mwtag7tylwtkuhxqyyvchsvndaxggv27pnu6du4gpz53jwtfr9y2jy",
    balance: "50000000000000000",
    tokenSymbol: "ULNK",
  },
};
