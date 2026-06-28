import { useCallback, useEffect, useRef, useState } from "react";
import { AlertTriangle, Gauge, Pause, Play, RotateCcw } from "lucide-react";
import { CANVAS } from "./mazeData.js";
import { COLORS, PALETTE } from "./theme.js";
import { drawSimulationCanvas, renderMazeBitmap } from "./canvasRenderer.js";
import {
  angle,
  angleDiff,
  createSimulation,
  fireNode,
  lerpAngle,
  trimLog,
  uiSnapshot,
} from "./mazeLogic.js";
import { ProgramDiagram } from "./components/ProgramBlocks.jsx";
import { DecisionPanel, LessonPanel, LogPanel, Panel, Pill, ScoreBox, SensorPanel, VariablesPanel } from "./components/Panels.jsx";

const SEEK_STEP = 1 / 60;
const KEY_SEEK_SMALL = 1;
const KEY_SEEK_LARGE = 5;

function formatTime(seconds) {
  const safe = Math.max(0, seconds || 0);
  const mins = Math.floor(safe / 60);
  const secs = safe - mins * 60;
  return `${mins}:${secs.toFixed(1).padStart(4, "0")}`;
}

function advanceSimulation(state, dtEff) {
  if (!state || state.done || dtEff <= 0) return;
  const leg = state.legs[state.li];
  const pts = leg.pts;

  state.time += dtEff;
  if (state.phase === "move") {
    const a = pts[state.vi];
    const b = pts[state.vi + 1];
    const len = Math.hypot(b[0] - a[0], b[1] - a[1]);
    state.t += (leg.base * dtEff) / len;
    state.micro = "FOLLOW";
    state.curPat = [0, 1, 0];
    state.curAct = "S";

    if (state.t >= 1) {
      state.t = 1;
      state.x = b[0];
      state.y = b[1];
      const arr = state.vi + 1;
      const isNode = arr in leg.evt;
      if (isNode) {
        fireNode(leg.evt[arr], state);
        state.micro = "READ";
      }

      if (arr >= pts.length - 1) {
        if (state.li < state.legs.length - 1) {
          state.returnStartTime = state.time;
          state.li += 1;
          state.vi = 0;
          state.t = 0;
          state.phase = "move";
          const nextPts = state.legs[state.li].pts;
          state.x = nextPts[0][0];
          state.y = nextPts[0][1];
          state.heading = angle(nextPts[0], nextPts[1]);
          state.log.unshift({ t: state.time, m: "탐사 완료 -> MODE=RETURN -> MOVELOG 역재생 시작" });
          trimLog(state);
        } else {
          state.done = true;
          state.phase = "done";
          state.micro = "DONE";
        }
      } else {
        const incoming = angle(pts[state.vi], pts[arr]);
        const outgoing = angle(pts[arr], pts[arr + 1]);
        const turn = Math.abs(angleDiff(incoming, outgoing)) > 0.4;
        const decision = isNode ? leg.decisions[leg.evt[arr]] : null;
        const isDecisionNode = decision && (decision.kind === "JCT" || decision.kind === "LEAF");
        const seen = decision && state.seenJ[decision.node];
        const needPeek = decision && decision.pattern[0] === 1 && decision.pattern[2] === 1 && !seen;
        const fDiscover = leg.mode === "EXPLORE" && decision && decision.node === "F" && leg.seq[arr + 1] === "E" && !state.eDiscovered;

        if (decision && decision.kind === "JCT") state.seenJ[decision.node] = true;
        state.turnFrom = incoming;
        state.turnTo = outgoing;
        state.turnNeeded = turn;
        state.pendVi = arr;

        if (fDiscover) {
          state.eDiscovered = true;
          state.phase = "discover";
          state.discT = 0.55;
          state.discLogged = false;
          state.micro = "DISCOVER";
        } else if (leg.mode === "EXPLORE" && needPeek) {
          state.phase = "peek";
          state.peekT = 0;
          state.peekDir = 1;
          state.peekHeading = incoming;
          state.peekClassified = false;
          state.micro = "PEEK";
        } else if (leg.mode === "EXPLORE" && isDecisionNode) {
          state.phase = "scan";
          state.scanT = 0.4;
          state.scanWig = 0;
          state.micro = "DECIDE";
        } else if (turn) {
          state.phase = "turn";
          state.turnT = 0;
          state.turnDur = Math.max(0.14, (Math.abs(angleDiff(incoming, outgoing)) / Math.PI) * 0.38);
          state.micro = "TURN";
        } else {
          state.vi = arr;
          state.t = 0;
        }
      }
    } else {
      state.x = a[0] + (b[0] - a[0]) * state.t;
      state.y = a[1] + (b[1] - a[1]) * state.t;
      state.heading = angle(a, b);
    }
  } else if (state.phase === "scan") {
    state.scanT -= dtEff;
    state.scanWig += dtEff * 11;
    state.heading = state.turnFrom + Math.sin(state.scanWig) * 0.32;
    state.micro = "DECIDE";
    if (state.scanT <= 0) {
      if (state.turnNeeded) {
        state.phase = "turn";
        state.turnT = 0;
        state.turnDur = Math.max(0.14, (Math.abs(angleDiff(state.turnFrom, state.turnTo)) / Math.PI) * 0.38);
        state.micro = "TURN";
      } else {
        state.vi = state.pendVi;
        state.t = 0;
        state.phase = "move";
        state.heading = state.turnTo;
      }
    }
  } else if (state.phase === "peek") {
    const joint = pts[state.pendVi];
    const hx = Math.cos(state.peekHeading);
    const hy = Math.sin(state.peekHeading);
    state.peekT = Math.max(0, Math.min(1, state.peekT + dtEff * 2.4 * state.peekDir));
    state.x = joint[0] + hx * 42 * state.peekT;
    state.y = joint[1] + hy * 42 * state.peekT;
    state.heading = state.peekHeading;
    state.micro = "PEEK";

    if (state.peekDir > 0 && state.peekT >= 1) {
      if (!state.peekClassified) {
        state.peekClassified = true;
        const decision = leg.decisions[leg.evt[state.pendVi]];
        state.peekDtype = decision.pattern[1] === 1;
        state.peekTypes[decision.node] = state.peekDtype ? "D" : "B";
        state.log.unshift({
          t: state.time,
          m: state.peekDtype ? "111 -> peek 직진 -> 010 -> D형(십자): 직진 먼저" : "111 -> peek 직진 -> 000 -> B형(T): 후진 후 좌선우선",
        });
        trimLog(state);
      }
      if (state.peekDtype && !state.turnNeeded) {
        state.vi = state.pendVi;
        state.t = 0.28;
        state.phase = "move";
      } else {
        state.peekDir = -1;
      }
    } else if (state.peekDir < 0 && state.peekT <= 0) {
      if (state.turnNeeded) {
        state.phase = "turn";
        state.turnT = 0;
        state.turnDur = Math.max(0.14, (Math.abs(angleDiff(state.turnFrom, state.turnTo)) / Math.PI) * 0.38);
        state.micro = "TURN";
      } else {
        state.vi = state.pendVi;
        state.t = 0;
        state.phase = "move";
        state.heading = state.turnTo;
      }
    }
  } else if (state.phase === "discover") {
    const joint = pts[state.pendVi];
    state.x = joint[0];
    state.y = joint[1];
    state.heading = state.turnFrom;
    state.micro = "DISCOVER";
    if (!state.discLogged) {
      state.discLogged = true;
      state.log.unshift({ t: state.time, m: "좌선우선으로 F까지 확인 -> F 분기 발견 -> E 복귀 -> 5부터 처리" });
      trimLog(state);
    }
    state.discT -= dtEff;
    if (state.discT <= 0) {
      state.eDiscoveryResolved = true;
      state.phase = "turn";
      state.turnT = 0;
      state.turnDur = Math.max(0.16, (Math.abs(angleDiff(state.turnFrom, state.turnTo)) / Math.PI) * 0.4);
      state.micro = "TURN";
    }
  } else if (state.phase === "turn") {
    state.turnT += dtEff / state.turnDur;
    state.micro = "TURN";
    if (state.turnT >= 1) {
      state.heading = state.turnTo;
      state.vi = state.pendVi;
      state.t = 0;
      state.phase = "move";
    } else {
      state.heading = lerpAngle(state.turnFrom, state.turnTo, state.turnT);
    }
  }
}

function seekSimulationState(targetTime) {
  const state = createSimulation();
  let remaining = Math.max(0, targetTime);
  while (remaining > 0 && !state.done) {
    const step = Math.min(SEEK_STEP, remaining);
    advanceSimulation(state, step);
    remaining -= step;
  }
  return state;
}

function computeTimeline() {
  const state = createSimulation();
  while (!state.done && state.time < 600) advanceSimulation(state, SEEK_STEP);
  return {
    totalTime: state.time,
    returnStartTime: state.returnStartTime || state.time,
  };
}

export default function App() {
  const initialSimRef = useRef(null);
  if (!initialSimRef.current) initialSimRef.current = createSimulation();
  const canvasRef = useRef(null);
  const bitmapRef = useRef(null);
  const simRef = useRef(initialSimRef.current);
  const rafRef = useRef(null);
  const runningRef = useRef(false);
  const speedRef = useRef(1);
  const cfgRef = useRef("B");
  const lastTsRef = useRef(0);
  const playheadRef = useRef(0);
  const timelineRef = useRef({ totalTime: 0, returnStartTime: 0 });
  const obstacleUntilRef = useRef(0);

  const [running, setRunning] = useState(false);
  const [cfg, setCfg] = useState("B");
  const [speed, setSpeed] = useState(1);
  const [ui, setUi] = useState(() => uiSnapshot(initialSimRef.current));
  const [playheadTime, setPlayheadTime] = useState(0);
  const [timeline, setTimeline] = useState({ totalTime: 0, returnStartTime: 0 });

  const renderState = useCallback((state, nextPlayhead) => {
    const obstacleActive = performance.now() < obstacleUntilRef.current && state.legs[state.li].mode === "EXPLORE";
    if (obstacleActive) {
      state.phase = "obstacle";
      state.micro = "OBSTACLE";
      state.log = [{ t: state.time, m: "초음파 감지 -> 일시 시연(재생바에는 기록하지 않음)" }, ...state.log].slice(0, 16);
    }

    const sensors = drawSimulationCanvas(canvasRef.current, bitmapRef.current, state, cfgRef.current);
    if (sensors) {
      state.bits = sensors.bits;
      state.refl = sensors.refl;
    }

    simRef.current = state;
    playheadRef.current = nextPlayhead;
    setPlayheadTime(nextPlayhead);
    setUi(uiSnapshot(state));
  }, []);

  const renderAt = useCallback(
    (time) => {
      const total = timelineRef.current.totalTime || 0;
      const target = Math.max(0, Math.min(time, total || time));
      const state = seekSimulationState(target);
      renderState(state, target);
      return state;
    },
    [renderState],
  );

  const resetSim = useCallback(
    (startAfterReset = false) => {
      obstacleUntilRef.current = 0;
      lastTsRef.current = 0;
      renderAt(0);
      runningRef.current = startAfterReset;
      setRunning(startAfterReset);
    },
    [renderAt],
  );

  useEffect(() => {
    bitmapRef.current = renderMazeBitmap();
    const nextTimeline = computeTimeline();
    timelineRef.current = nextTimeline;
    setTimeline(nextTimeline);
    resetSim(false);

    const loop = (ts) => {
      const last = lastTsRef.current || ts;
      let dt = (ts - last) / 1000;
      lastTsRef.current = ts;
      dt = Math.min(dt, 0.05);
      if (runningRef.current) {
        const total = timelineRef.current.totalTime;
        const nextPlayhead = Math.min(total, playheadRef.current + dt * speedRef.current);
        renderAt(nextPlayhead);
        if (nextPlayhead >= total) {
          runningRef.current = false;
          setRunning(false);
        }
      } else if (performance.now() < obstacleUntilRef.current && simRef.current) {
        renderAt(playheadRef.current);
      }
      rafRef.current = requestAnimationFrame(loop);
    };

    rafRef.current = requestAnimationFrame(loop);
    return () => cancelAnimationFrame(rafRef.current);
  }, [renderAt, resetSim]);

  const togglePlayback = useCallback(() => {
    if (runningRef.current) {
      runningRef.current = false;
      setRunning(false);
      return;
    }

    if (playheadRef.current >= timelineRef.current.totalTime) {
      renderAt(0);
    } else {
      obstacleUntilRef.current = 0;
    }
    runningRef.current = true;
    setRunning(true);
    lastTsRef.current = 0;
  }, [renderAt]);

  const onToggle = () => togglePlayback();

  const onReset = () => resetSim(false);

  const onCfg = (nextCfg) => {
    cfgRef.current = nextCfg;
    setCfg(nextCfg);
    if (simRef.current) renderState(simRef.current, playheadRef.current);
  };

  const onSpeed = (event) => {
    const value = Number.parseFloat(event.target.value);
    speedRef.current = value;
    setSpeed(value);
  };

  const onSeek = (event) => {
    const value = Number.parseFloat(event.target.value);
    obstacleUntilRef.current = 0;
    runningRef.current = false;
    setRunning(false);
    lastTsRef.current = 0;
    renderAt(value);
  };

  const onSeekStart = () => {
    obstacleUntilRef.current = 0;
    runningRef.current = false;
    setRunning(false);
    lastTsRef.current = 0;
  };

  const seekBy = useCallback(
    (delta) => {
      const total = timelineRef.current.totalTime || 0;
      const next = Math.max(0, Math.min(total, playheadRef.current + delta));
      obstacleUntilRef.current = 0;
      runningRef.current = false;
      setRunning(false);
      lastTsRef.current = 0;
      renderAt(next);
    },
    [renderAt],
  );

  useEffect(() => {
    const onKeyDown = (event) => {
      const target = event.target;
      const tagName = target?.tagName;
      const isEditable = target?.isContentEditable || tagName === "TEXTAREA" || tagName === "SELECT" || (tagName === "INPUT" && target.type !== "range");
      if (event.defaultPrevented || isEditable) return;

      if (event.code === "Space" || event.key === " ") {
        event.preventDefault();
        if (!event.repeat) togglePlayback();
      } else if (event.key === "ArrowLeft") {
        event.preventDefault();
        seekBy(-KEY_SEEK_SMALL);
      } else if (event.key === "ArrowRight") {
        event.preventDefault();
        seekBy(KEY_SEEK_SMALL);
      } else if (event.key === "ArrowUp") {
        event.preventDefault();
        seekBy(KEY_SEEK_LARGE);
      } else if (event.key === "ArrowDown") {
        event.preventDefault();
        seekBy(-KEY_SEEK_LARGE);
      }
    };

    window.addEventListener("keydown", onKeyDown);
    return () => window.removeEventListener("keydown", onKeyDown);
  }, [seekBy, togglePlayback]);

  if (!ui) {
    return <div className="loading">로딩 중...</div>;
  }

  const mode = ui.mode;
  const on = (...states) => states.includes(ui.micro);
  const patStr = ui.pat.join("");
  const totalTime = timeline.totalTime || 0;
  const returnPercent = totalTime ? Math.min(100, Math.max(0, (timeline.returnStartTime / totalTime) * 100)) : 0;

  return (
    <main className="app">
      <header className="app-header">
        <div>
          <h1>
            EV3 Maze Lab <span>ver5</span>
          </h1>
          <p>웹 시뮬레이터 + ev3dev 로봇 코드 · EXPLORE / RETURN 알고리즘</p>
        </div>
        <div className="mode-pills">
          <Pill active={mode === "EXPLORE"} color={PALETTE.flow}>EXPLORE</Pill>
          <Pill active={mode === "RETURN"} color={PALETTE.action}>RETURN</Pill>
          <Pill active={ui.specPathOk} color={PALETTE.my}>SPEC PATH</Pill>
          {ui.done && <Pill active color={PALETTE.action}>완료</Pill>}
        </div>
      </header>

      <ProgramDiagram mode={mode} on={on} patStr={patStr} act={ui.act} peek={ui.peek} tok={ui.tok} />

      <section className="workspace">
        <div className="maze-panel">
          <canvas ref={canvasRef} width={Math.ceil(CANVAS.width)} height={Math.ceil(CANVAS.height)} />
          <div className="playback-bar" style={{ "--return-start": `${returnPercent}%` }}>
            <div className="playback-meta">
              <span>{formatTime(playheadTime)}</span>
              <b>왕복 전체 재생</b>
              <span>{formatTime(totalTime)}</span>
            </div>
            <div className="playback-track">
              <input
                type="range"
                min={0}
                max={totalTime || 0}
                step={0.05}
                value={Math.min(playheadTime, totalTime)}
                onPointerDown={onSeekStart}
                onChange={onSeek}
                aria-label="왕복 전체 재생 위치"
              />
              <i aria-hidden="true" />
            </div>
            <div className="playback-labels">
              <span>{"EXPLORE 0 -> 7"}</span>
              <span>{"RETURN 7 -> 0"}</span>
            </div>
            <div className="keyboard-hint">Space 재생/정지 · ←/→ 1초 이동 · ↑/↓ 5초 이동</div>
          </div>
          <div className="controls maze-controls">
            <button className="control-button primary" type="button" onClick={onToggle}>
              {running ? <Pause size={17} /> : <Play size={17} />}
              {running ? "정지" : ui.done ? "재생" : "실행"}
            </button>
            <button className="control-button" type="button" onClick={onReset}>
              <RotateCcw size={17} />
              리셋
            </button>
            <button className="control-button warning" type="button" onClick={() => { obstacleUntilRef.current = performance.now() + 1600; renderAt(playheadRef.current); }} disabled={mode !== "EXPLORE"}>
              <AlertTriangle size={17} />
              장애물 주입
            </button>
            <div className="segmented" aria-label="센서 배치">
              {["B", "C"].map((name) => (
                <button type="button" key={name} data-active={cfg === name ? "true" : "false"} onClick={() => onCfg(name)}>
                  배치 {name}
                </button>
              ))}
            </div>
            <label className="speed-control">
              <Gauge size={16} />
              <span>x{speed.toFixed(1)}</span>
              <input type="range" min={0.1} max={3} step={0.1} value={speed} onChange={onSpeed} />
            </label>
          </div>
        </div>

        <aside className="side-panel">
          <LessonPanel ui={ui} />
          <Panel title="채점 (구간별 도달 직전 방문 노드)">
            <div className="score-grid">
              <ScoreBox label="출발 -> 도착 7" value={ui.scoreOut} done={ui.scoreOut > 0} />
              <ScoreBox label="도착 -> 출발 0" value={ui.scoreBack} done={ui.scoreBack > 0} />
            </div>
            <div className="panel-note">각 구간 끝점 도달 시 8/8 이면 만점입니다.</div>
          </Panel>
          <SensorPanel cfg={cfg} ui={ui} />
          <DecisionPanel ui={ui} />
          <VariablesPanel ui={ui} mode={mode} />
          <LogPanel ui={ui} />
        </aside>
      </section>
    </main>
  );
}

document.documentElement.style.setProperty("--app-bg", COLORS.bg);
