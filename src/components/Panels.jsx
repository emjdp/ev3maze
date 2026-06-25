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

export function LessonPanel({ ui }) {
  const lesson = ui.debug?.lesson;
  if (!lesson) return null;
  const flow = ["센서 읽기", "분기 판단", "회전 기록", "전체 방문", "기록 복귀"];

  return (
    <Panel title="지금 알고리즘이 하는 일">
      <div className="lesson-mode">{lesson.modeText}</div>
      <div className="lesson-card">
        <span>{lesson.concept}</span>
        <strong>{lesson.headline}</strong>
      </div>
      <div className="lesson-flow" aria-label="전체 흐름">
        {flow.map((label, i) => (
          <span key={label} data-active={lesson.activeStep === i ? "true" : "false"}>
            {i + 1}. {label}
          </span>
        ))}
      </div>
      <ol className="lesson-steps">
        {lesson.plainSteps.map((step) => (
          <li key={step}>{step}</li>
        ))}
      </ol>
      <div className="why-line">
        <span>하드코딩이 아닌 이유</span>
        <b>{lesson.whyItMatters}</b>
      </div>
      <div className="glossary-grid">
        <Glossary term="pattern" text="세 센서가 본 선/바닥을 010처럼 묶은 값" />
        <Glossary term="seen" text="이 분기를 전에 본 적 있는지 표시" />
        <Glossary term="path[]" text="로봇이 실제로 한 회전 기록" />
        <Glossary term="stack" text="지금 먼저 끝내야 할 분기 흐름" />
      </div>
    </Panel>
  );
}

export function DecisionPanel({ ui }) {
  const debug = ui.debug;
  if (!debug) return null;
  const { decision, pathState } = debug;
  const refl = ["L", "C", "R"].map((key) => ui.refl[key]);
  const bits = ui.bits;
  const sensorPattern = bits.join("");

  return (
    <Panel title="판단 로직">
      <div className="decision-head">
        <div>
          <span>NODE</span>
          <strong>{decision.node}</strong>
        </div>
        <div>
          <span>PATTERN</span>
          <strong>{decision.patternNumber}</strong>
        </div>
        <div>
          <span>ACTION</span>
          <strong>{decision.actionLabel}</strong>
        </div>
      </div>
      <div className="formula-row">
        <span>reflection &lt; {debug.threshold}</span>
        <b>
          {refl.map((v, i) => `${v}->${bits[i]}`).join(" / ")}
        </b>
      </div>
      <div className="formula-row">
        <span>pattern</span>
        <b>{bits[0]}*100 + {bits[1]}*10 + {bits[2]} = {sensorPattern}</b>
      </div>
      <div className="candidate-table">
        <div className="candidate-table__head">
          <span>방향</span>
          <span>연결</span>
          <span>우선</span>
          <span>가봄</span>
          <span>선택</span>
        </div>
        {decision.candidates.map((row) => (
          <div className="candidate-row" data-selected={row.selected ? "true" : "false"} key={row.dir}>
            <b>{row.label}</b>
            <span>{row.node || "-"}</span>
            <span>{row.open ? row.priority : "-"}</span>
            <span>{row.done ? "예" : "아니오"}</span>
            <span>{row.selected ? "선택" : "-"}</span>
          </div>
        ))}
      </div>
      <div className="reason-line">{decision.reason}</div>
      {ui.mode === "RETURN" && (
        <div className="return-line">
          read i={pathState.currentReadIndex} · next i={pathState.nextReadIndex} · {pathState.sourceTokenName || "-"} -&gt; {pathState.invertedTokenName || "-"}
        </div>
      )}
    </Panel>
  );
}

export function VariablesPanel({ ui, mode }) {
  const debug = ui.debug;
  return (
    <Panel title="변수 (MOVELOG)">
      <InfoRow label="MODE" value={mode} highlight={mode === "EXPLORE" ? "flow" : "action"} />
      <InfoRow label="micro" value={ui.micro} />
      {debug && (
        <>
          <InfoRow label="기록된 회전 수(pathLen)" value={debug.pathState.pathLen} />
          <InfoRow label="다음 기록 위치(writeIdx)" value={debug.pathState.currentWriteIndex ?? "-"} />
          <InfoRow label="복귀 읽기 위치(readIdx)" value={debug.pathState.currentReadIndex ?? "-"} />
          <div className="memory-grid">
            <div className="memory-grid__head">
              <span>분기</span>
              <span>본 적</span>
              <span>형태</span>
              <span>가본 방향</span>
            </div>
            {debug.memory.junctions.map((row) => (
              <div className="memory-row" key={row.id} data-seen={row.seen ? "true" : "false"}>
                <b>{row.id}</b>
                <span>{row.seen ? "예" : "아니오"}</span>
                <span>{row.type}</span>
                <span>{["L", "S", "R"].map((dir) => (row.mask[dir] ? dir : "-")).join("")}</span>
              </div>
            ))}
          </div>
          <div className="stack-strip">
            <span>stack[]</span>
            {debug.memory.stack.length ? debug.memory.stack.map((node) => <b key={node}>{node}</b>) : <b>-</b>}
          </div>
        </>
      )}
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

function Glossary({ term, text }) {
  return (
    <div>
      <b>{term}</b>
      <span>{text}</span>
    </div>
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
