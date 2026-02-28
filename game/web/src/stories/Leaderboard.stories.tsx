import type { Meta, StoryObj } from "@storybook/react";
import Leaderboard from "../components/Leaderboard";
import "../App.css";

const meta: Meta<typeof Leaderboard> = {
  title: "Panels/Leaderboard",
  component: Leaderboard,
  decorators: [(Story) => <div style={{ width: 320, background: "#12121e" }}><Story /></div>],
};

export default meta;
type Story = StoryObj<typeof Leaderboard>;

export const Empty: Story = {
  args: { entries: [], currentPlayerId: null },
};

export const ThreePlayers: Story = {
  args: {
    entries: [
      { rank: 1, id: "alice", score: 2500000000000000000 },
      { rank: 2, id: "bob", score: 1000000000000000000 },
      { rank: 3, id: "charlie", score: 0 },
    ],
    currentPlayerId: "bob",
  },
};

export const SixPlayers: Story = {
  args: {
    entries: [
      { rank: 1, id: "alice", score: 3500000000000000000 },
      { rank: 2, id: "bob", score: 2000000000000000000 },
      { rank: 3, id: "charlie", score: 1500000000000000000 },
      { rank: 4, id: "dana", score: 1000000000000000000 },
      { rank: 5, id: "eve", score: 500000000000000000 },
      { rank: 6, id: "frank", score: 0 },
    ],
    currentPlayerId: "dana",
  },
};

export const SinglePlayer: Story = {
  args: {
    entries: [{ rank: 1, id: "solo_player", score: 750000000000000000 }],
    currentPlayerId: "solo_player",
  },
};
