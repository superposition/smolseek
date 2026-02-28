import type { Meta, StoryObj } from "@storybook/react";
import BidPanel from "../components/BidPanel";
import type { Cache, GameState } from "../types";
import "../App.css";

const mockCache: Cache = {
  id: "cache_3",
  x: 2.0,
  y: -1.0,
  z: 0.0,
  value: "1000000000000000000",
  collected: false,
  collected_by: null,
};


const lobbyState: GameState = {
  phase: "LOBBY",
  current_round: 0,
  total_rounds: 5,
  players: {},
  caches: {},
  leaderboard: [],
  phase_deadline: null,
};

const playState: GameState = {
  ...lobbyState,
  phase: "PLAY",
  current_round: 2,
};

const endState: GameState = {
  ...lobbyState,
  phase: "END",
  current_round: 5,
};

const meta: Meta<typeof BidPanel> = {
  title: "Panels/BidPanel",
  component: BidPanel,
  decorators: [(Story) => <div style={{ width: 320, background: "#12121e" }}><Story /></div>],
};

export default meta;
type Story = StoryObj<typeof BidPanel>;

export const LobbyPhase: Story = {
  args: { gameState: lobbyState, selectedCache: null, onSubmitBid: () => {}, disabled: false },
};

export const PlayNoCacheSelected: Story = {
  args: { gameState: playState, selectedCache: null, onSubmitBid: () => {}, disabled: false },
};

export const PlayWithCacheSelected: Story = {
  args: {
    gameState: playState,
    selectedCache: mockCache,
    onSubmitBid: (cacheId, amount) => alert(`Bid ${amount} on ${cacheId}`),
    disabled: false,
  },
};

export const PlayDisabledNotJoined: Story = {
  args: {
    gameState: playState,
    selectedCache: mockCache,
    onSubmitBid: () => {},
    disabled: true,
  },
};

export const GameOver: Story = {
  args: { gameState: endState, selectedCache: null, onSubmitBid: () => {}, disabled: false },
};
