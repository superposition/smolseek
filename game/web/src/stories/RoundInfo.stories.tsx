import type { Meta, StoryObj } from "@storybook/react";
import RoundInfo from "../components/RoundInfo";
import type { GameState, RoundResult } from "../types";
import "../App.css";

const baseState: GameState = {
  phase: "LOBBY",
  current_round: 0,
  total_rounds: 5,
  players: {},
  caches: {},
  leaderboard: [],
  phase_deadline: null,
};

const meta: Meta<typeof RoundInfo> = {
  title: "Panels/RoundInfo",
  component: RoundInfo,
  decorators: [(Story) => <div style={{ width: 320, background: "#12121e" }}><Story /></div>],
};

export default meta;
type Story = StoryObj<typeof RoundInfo>;

export const Lobby: Story = {
  args: { gameState: baseState, roundResult: null, navStatus: null },
};

export const DepositPhase: Story = {
  args: {
    gameState: { ...baseState, phase: "DEPOSIT", phase_deadline: Date.now() / 1000 + 45 },
    roundResult: null,
    navStatus: null,
  },
};

export const PlayBiddingOpen: Story = {
  args: {
    gameState: {
      ...baseState,
      phase: "PLAY",
      current_round: 3,
      total_rounds: 5,
      phase_deadline: Date.now() / 1000 + 22,
    },
    roundResult: null,
    navStatus: null,
  },
};

export const RobotNavigating: Story = {
  args: {
    gameState: { ...baseState, phase: "PLAY", current_round: 2, total_rounds: 5 },
    roundResult: null,
    navStatus: { status: "navigating", distance_remaining: 1.4 },
  },
};

export const RoundWon: Story = {
  args: {
    gameState: { ...baseState, phase: "PLAY", current_round: 2, total_rounds: 5 },
    roundResult: {
      number: 2,
      bids: [],
      winner_id: "alice",
      target_cache_id: "cache_3",
      nav_status: "arrived",
    },
    navStatus: null,
  },
};

export const GameOver: Story = {
  args: {
    gameState: { ...baseState, phase: "END", current_round: 5 },
    roundResult: null,
    navStatus: null,
  },
};
