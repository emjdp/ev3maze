import { NUMERIC_NODES, NODES } from "../mazeData.js";
import { TOKNAME } from "../mazeLogic.js";

export function Pill({ active, color, children }) {
  return (
    <span className="pill" style={{ "--pill-color": color }} data-active={active ? "true" : "false"}>
      {children}
    </span>
  );
}

export function Panel({ title, children }) {
  return (
    <section className="panel">
      <h2>{title}</h2>
      {children}
    </section>
  );
}

export function ScoreBox({ label, value, done }) {
  const full = value >= 8;
  return (
    <div className="score-box" data-done={done ? "true" : "false"} data-full={full ? "true" : "false"}>
      <span>{label}</span>
      <strong>{done ? `${value}/8` : "-"}</strong>
    </div>
  );
}

export function SensorPanel({ cfg, ui }) {
  return (
    <Panel title={`컬러센서 (배치 ${cfg}) - 패턴 ${ui.bits.join(" ")}`}>
      <div className="sensor-grid">
        {["L", "C", "R"].map((key, i) => {
          const refl = ui.refl[key];
          const bit = ui.bits[i];
          return (
            <div className="sensor-card" key={key}>
              <span>{key}</span>
              <div className="sensor-bar">
                <i style={{ height: `${refl}%` }} data-bit={bit ? "true" : "false"} />
              </div>
              <strong data-bit={bit ? "true" : "false"}>{bit}</strong>
            </div>
          );
        })}
      </div>
    </Panel>
  );
}

export function VariablesPanel({ ui, mode }) {
  return (
    <Panel title="변수 (MOVELOG)">
      <InfoRow label="MODE" value={mode} highlight={mode === "EXPLORE" ? "flow" : "action"} />
      <InfoRow label="micro" value={ui.micro} />
      <div className="mini-label">path 기록 (탐사)</div>
      <TokenStrip arr={ui.moveLog} count={ui.expRec} active={mode === "EXPLORE"} />
      <div className="mini-label spaced">역재생 (복귀, 반전)</div>
      <TokenStrip arr={ui.revInv} count={ui.retRec} active={mode === "RETURN"} />
      <div className="node-strip">
        {NUMERIC_NODES.map((node) => {
          const visited = ui.visited.includes(node);
          return (
            <span key={node} data-kind={NODES[node].kind} data-visited={visited ? "true" : "false"}>
              {node}
            </span>
          );
        })}
      </div>
      <div className="panel-note">수집 {ui.collected}/8 · {ui.time.toFixed(1)}s</div>
    </Panel>
  );
}

export function LogPanel({ ui }) {
  return (
    <Panel title="실행 로그">
      <div className="log-list">
        {ui.log.map((line, i) => (
          <div key={`${line.t}-${i}`} data-current={i === 0 ? "true" : "false"}>
            <time>{line.t.toFixed(1)}</time>
            <span>{line.m}</span>
          </div>
        ))}
      </div>
    </Panel>
  );
}

export function InfoRow({ label, value, highlight }) {
  return (
    <div className="info-row" data-highlight={highlight || ""}>
      <span>{label}</span>
      <strong>{value}</strong>
    </div>
  );
}

export function TokenStrip({ arr, count, active }) {
  return (
    <div className="token-strip">
      {arr.map((token, i) => (
        <span key={`${token}-${i}`} data-shown={i < count ? "true" : "false"} data-active={active ? "true" : "false"} data-current={i === count - 1 && active ? "true" : "false"}>
          {TOKNAME[token]}
        </span>
      ))}
    </div>
  );
}
