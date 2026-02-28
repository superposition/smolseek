/**
 * EscrowPool — Live escrow balance + address with copy button.
 */

import { useState } from "react";

interface EscrowPoolProps {
  address: string;
  balance: string; // wei
  tokenSymbol?: string;
}

export default function EscrowPool({
  address,
  balance,
  tokenSymbol = "MON",
}: EscrowPoolProps) {
  const [copied, setCopied] = useState(false);
  const displayBalance = (parseInt(balance) / 1e18).toFixed(4);
  const shortAddr = address.length > 20
    ? `${address.slice(0, 12)}...${address.slice(-8)}`
    : address;

  const handleCopy = () => {
    navigator.clipboard.writeText(address);
    setCopied(true);
    setTimeout(() => setCopied(false), 1500);
  };

  return (
    <div className="panel">
      <h3>Escrow Pool</h3>
      <div className="escrow-pool">
        <div className="escrow-row">
          <span className="escrow-label">Balance</span>
          <span className="escrow-value">{displayBalance} {tokenSymbol}</span>
        </div>
        <div className="escrow-row">
          <span className="escrow-label">Address</span>
          <div className="escrow-address">
            <span className="escrow-value">{shortAddr}</span>
            <button className="escrow-copy" onClick={handleCopy}>
              {copied ? "copied" : "copy"}
            </button>
          </div>
        </div>
      </div>
    </div>
  );
}
