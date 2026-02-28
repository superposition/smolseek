import type { Meta, StoryObj } from "@storybook/react";
import CacheCard from "../components/CacheCard";
import type { Cache } from "../types";
import "../App.css";

const meta: Meta<typeof CacheCard> = {
  title: "Panels/CacheCard",
  component: CacheCard,
  decorators: [(Story) => <div style={{ width: 320, background: "#12121e", padding: 16 }}><Story /></div>],
};

export default meta;
type Story = StoryObj<typeof CacheCard>;

const available: Cache = {
  id: "cache_3",
  x: 2.0,
  y: -1.5,
  z: 0.0,
  value: "1000000000000000000",
  collected: false,
  collected_by: null,
};

const highValue: Cache = {
  id: "cache_7",
  x: -3.2,
  y: 4.1,
  z: 0.0,
  value: "5000000000000000000",
  collected: false,
  collected_by: null,
};

const collected: Cache = {
  id: "cache_1",
  x: 0.5,
  y: 0.8,
  z: 0.0,
  value: "500000000000000000",
  collected: true,
  collected_by: "alice",
};

export const Available: Story = {
  args: { cache: available, onClick: (id) => alert(`Clicked ${id}`) },
};

export const Selected: Story = {
  args: { cache: available, selected: true },
};

export const HighValue: Story = {
  args: { cache: highValue },
};

export const Collected: Story = {
  args: { cache: collected },
};
