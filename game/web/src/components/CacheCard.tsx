/**
 * CacheCard — Display a single cache with id, value, coordinates, and status.
 */

import type { Cache } from "../types";

interface CacheCardProps {
  cache: Cache;
  selected?: boolean;
  onClick?: (id: string) => void;
}

export default function CacheCard({ cache, selected, onClick }: CacheCardProps) {
  const value = (parseInt(cache.value) / 1e18).toFixed(2);
  const classes = [
    "cache-card",
    selected ? "cache-selected" : "",
    cache.collected ? "cache-collected" : "",
  ].filter(Boolean).join(" ");

  return (
    <div
      className={classes}
      onClick={() => onClick?.(cache.id)}
      style={{ cursor: onClick ? "pointer" : undefined }}
    >
      <div className="cache-card-top">
        <span className="cache-card-id">{cache.id}</span>
        <span className="cache-card-value">{value} MON</span>
      </div>
      <div className="cache-card-bottom">
        <span className="cache-card-coords">
          ({cache.x.toFixed(1)}, {cache.y.toFixed(1)})
        </span>
        <span className={`cache-card-status ${cache.collected ? "collected" : "available"}`}>
          {cache.collected ? `taken by ${cache.collected_by}` : "available"}
        </span>
      </div>
    </div>
  );
}
