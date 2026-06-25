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
import { LogPanel, Panel, Pill, ScoreBox, SensorPanel, VariablesPanel } from "./components/Panels.jsx";

export default function App() {
  const canvasRef = useRef(null);
  const bitmapRef = useRef(null);
  const simRef = useRef(null);
  const rafRef = useRef(null);
  const runningRef = useRef(false);
  const speedRef = useRef(1);
  const cfgRef = useRef("B");
  const obstacleRef = useRef(false);

  const [running, setRunning] = useState(false);
  const [cfg, setCfg] = useState("B");
  const [speed, setSpeed] = useState(1);
  const [ui, setUi] = useState(null);

  const pushUi = useCallback(() => {
    setUi(uiSnapshot(simRef.current));
  }, []);

  const drawFrame = useCallback(
    (dtEff) => {
      const state = simRef.current;
      if (!state) return;
      const leg = state.legs[state.li];
      const pts = leg.pts;

      if (!state.done && dtEff > 0) {
        state.time += dtEff;
        if (state.phase === "obstacle") {
          state.obsT -= dtEff;
          state.micro = "OBSTACLE";
          if (state.obsT <= 0) {
            state.phase = "move";
            state.micro = "FOLLOW";
            state.log.unshift({ t: state.time, m: "장애물 처리 완료 -> 추종 재개" });
            trimLog(state);
          }
        } else if (state.phase === "move") {
          if (obstacleRef.current && leg.mode === "EXPLORE") {
            obstacleRef.current = false;
            state.phase = "obstacle";
            state.obsT = 1.6;
            state.micro = "OBSTACLE";
            state.log.unshift({ t: state.time, m: "초음파 감지 -> Loop Interrupt LF -> Handle_Obstacle" });
            trimLog(state);
          } else {
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
                  runningRef.current = false;
                  setRunning(false);
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

      const sensors = drawSimulationCanvas(canvasRef.current, bitmapRef.current, state, cfgRef.current);
      if (sensors) {
        state.bits = sensors.bits;
        state.refl = sensors.refl;
      }
      pushUi();
    },
    [pushUi],
  );

  const resetSim = useCallback(
    (startAfterReset = false) => {
      obstacleRef.current = false;
      simRef.current = createSimulation();
      runningRef.current = startAfterReset;
      setRunning(startAfterReset);
      drawFrame(0);
    },
    [drawFrame],
  );

  useEffect(() => {
    bitmapRef.current = renderMazeBitmap();
    resetSim(false);

    const loop = (ts) => {
      const state = simRef.current;
      if (!state) {
        rafRef.current = requestAnimationFrame(loop);
        return;
      }
      const last = state.lastTs || ts;
      let dt = (ts - last) / 1000;
      state.lastTs = ts;
      dt = Math.min(dt, 0.05);
      drawFrame(runningRef.current && !state.done ? dt * speedRef.current : 0);
      rafRef.current = requestAnimationFrame(loop);
    };

    rafRef.current = requestAnimationFrame(loop);
    return () => cancelAnimationFrame(rafRef.current);
  }, [drawFrame, resetSim]);

  const onToggle = () => {
    if (running) {
      runningRef.current = false;
      setRunning(false);
      return;
    }

    if (simRef.current?.done) {
      resetSim(true);
    } else {
      runningRef.current = true;
      setRunning(true);
      if (simRef.current) simRef.current.lastTs = 0;
    }
  };

  const onReset = () => resetSim(false);

  const onCfg = (nextCfg) => {
    cfgRef.current = nextCfg;
    setCfg(nextCfg);
    drawFrame(0);
  };

  const onSpeed = (event) => {
    const value = Number.parseFloat(event.target.value);
    speedRef.current = value;
    setSpeed(value);
  };

  if (!ui) {
    return <div className="loading">로딩 중...</div>;
  }

  const mode = ui.mode;
  const on = (...states) => states.includes(ui.micro);
  const patStr = ui.pat.join("");

  return (
    <main className="app">
      <header className="app-header">
        <div>
          <h1>
            EV3 Lab 미로 시뮬레이터 <span>ver5</span>
          </h1>
          <p>단일패스 좌선우선 + 111 peek(B/D형 변별) · MOVELOG 역재생 복귀</p>
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
        </div>

        <aside className="side-panel">
          <Panel title="채점 (구간별 도달 직전 방문 노드)">
            <div className="score-grid">
              <ScoreBox label="출발 -> 도착 7" value={ui.scoreOut} done={ui.scoreOut > 0} />
              <ScoreBox label="도착 -> 출발 0" value={ui.scoreBack} done={ui.scoreBack > 0} />
            </div>
            <div className="panel-note">각 구간 끝점 도달 시 8/8 이면 만점입니다.</div>
          </Panel>
          <SensorPanel cfg={cfg} ui={ui} />
          <VariablesPanel ui={ui} mode={mode} />
          <LogPanel ui={ui} />
        </aside>
      </section>

      <footer className="controls">
        <button className="control-button primary" type="button" onClick={onToggle}>
          {running ? <Pause size={17} /> : <Play size={17} />}
          {running ? "정지" : ui.done ? "재생" : "실행"}
        </button>
        <button className="control-button" type="button" onClick={onReset}>
          <RotateCcw size={17} />
          리셋
        </button>
        <button className="control-button warning" type="button" onClick={() => { obstacleRef.current = true; }} disabled={mode !== "EXPLORE"}>
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
          <input type="range" min={0.3} max={3} step={0.1} value={speed} onChange={onSpeed} />
        </label>
      </footer>
    </main>
  );
}

document.documentElement.style.setProperty("--app-bg", COLORS.bg);
