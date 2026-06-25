import {
  ArrowLeftRight,
  Database,
  Download,
  Eye,
  GitBranch,
  Hand,
  MoveRight,
  Play,
  RotateCw,
  Route,
  ScanLine,
  Shuffle,
  Upload,
  Wrench,
} from "lucide-react";
import { PALETTE } from "../theme.js";
import { TOKNAME } from "../mazeLogic.js";

function IconSlot({ icon: Icon }) {
  return <Icon aria-hidden="true" size={18} strokeWidth={2.4} />;
}

function EvBlock({ color, name, icon, foot, active, width = 84 }) {
  return (
    <div className="ev-block" data-active={active ? "true" : "false"} style={{ "--block-color": color, "--block-width": `${width}px` }}>
      <div className="ev-block__head">{name}</div>
      <div className="ev-block__icon">
        <IconSlot icon={icon} />
      </div>
      <div className="ev-block__foot">{foot}</div>
    </div>
  );
}

function Conn() {
  return <div className="ev-conn" />;
}

function LoopFrame({ label, color, children }) {
  return (
    <div className="loop-frame" style={{ "--loop-color": color }}>
      <div className="loop-frame__label">{label}</div>
      <div className="loop-frame__body">{children}</div>
      <RotateCw className="loop-frame__repeat" size={14} aria-hidden="true" />
    </div>
  );
}

function SwitchBlock({ cases, activeCase }) {
  return (
    <div className="switch-block">
      {cases.map((cs, i) => (
        <div className="switch-row" data-active={activeCase === i ? "true" : "false"} key={cs.label}>
          <div className="switch-row__label">
            {activeCase === i ? <GitBranch size={12} /> : <MoveRight size={12} />}
            {cs.label}
          </div>
          <div className="switch-row__body">{cs.body}</div>
        </div>
      ))}
    </div>
  );
}

export function ProgramLegend() {
  return (
    <div className="program-legend">
      <Legend color={PALETTE.flow}>Flow</Legend>
      <Legend color={PALETTE.sensor}>Sensor</Legend>
      <Legend color={PALETTE.data}>Data</Legend>
      <Legend color={PALETTE.action}>Action</Legend>
      <Legend color={PALETTE.my}>My Block</Legend>
    </div>
  );
}

function Legend({ color, children }) {
  return (
    <span>
      <i style={{ background: color }} />
      {children}
    </span>
  );
}

export function ProgramDiagram({ mode, on, patStr, act, peek, tok }) {
  return (
    <section className="program-shell">
      <div className="program-shell__title">
        EV3-G 프로그램 - {mode === "EXPLORE" ? "탐사 루프 (EXPLORE)" : "복귀 루프 (RETURN)"}
      </div>
      {mode === "EXPLORE" ? <ExploreProgram on={on} patStr={patStr} act={act} peek={peek} /> : <ReturnProgram on={on} tok={tok} />}
      <ProgramLegend />
    </section>
  );
}

function ExploreProgram({ on, patStr, act, peek }) {
  const following = on("FOLLOW");
  const deciding = on("READ", "DECIDE", "TURN");
  const obstacle = on("OBSTACLE");
  const peekOn = on("PEEK");
  const pType = peek && peek.type;
  const discoverOn = on("DISCOVER");
  const rows = [
    ["010", "직진 S"],
    ["100", "좌회전 L"],
    ["110", "좌회전 L"],
    ["101", "좌회전 L"],
    ["111", "좌회전 L"],
    ["011", "직진 S"],
    ["001", "우회전 R"],
    ["000", "U턴 · 색기록"],
  ];

  return (
    <>
      <LoopFrame label="MAIN" color={PALETTE.flow}>
        <EvBlock color={PALETTE.start} name="Start" icon={Play} foot="" width={56} />
        <Conn />
        <EvBlock color={PALETTE.my} name="Follow_Until_Event" icon={Route} foot="LF loop" active={following} width={112} />
        <Conn />
        <SwitchBlock
          activeCase={obstacle ? 1 : 0}
          cases={[
            {
              label: "Event=0",
              body: (
                <div className="ev-flow-row">
                  <EvBlock color={PALETTE.my} name="Read_Pattern" icon={ScanLine} foot={patStr} active={deciding || peekOn} width={86} />
                  <Conn />
                  <div className="peek-box" data-active={peekOn ? "true" : "false"}>
                    <div className="peek-box__title">111(좌·우)? - Peek 직진</div>
                    <EvBlock color={PALETTE.sensor} name="Peek 직진" icon={Eye} foot="C센서 판독" active={peekOn} width={82} />
                    <PeekCase bits="010" label="D형(십자) - 직진 먼저" active={pType === "D"} />
                    <PeekCase bits="000" label="B형(T) - 후진·좌선" active={pType === "B"} />
                  </div>
                  <div className="pattern-table">
                    {rows.map(([bits, label]) => (
                      <div key={bits} data-active={deciding && patStr === bits ? "true" : "false"}>
                        <b>{bits}</b>
                        <span>{label}</span>
                      </div>
                    ))}
                  </div>
                  <EvBlock color={PALETTE.my} name="Turn_Until_Line" icon={RotateCw} foot={act} active={on("TURN")} width={92} />
                  <Conn />
                  <EvBlock color={PALETTE.flow} name="한 분기 우선" icon={Hand} foot="F까지->유턴" active={discoverOn} width={88} />
                  <Conn />
                  <EvBlock color={PALETTE.data} name="Array Write" icon={Download} foot="MOVELOG" active={on("READ")} width={78} />
                </div>
              ),
            },
            {
              label: "Event=1",
              body: <EvBlock color={PALETTE.my} name="Handle_Obstacle" icon={Wrench} foot="리프트(예약)" active={obstacle} width={104} />,
            },
          ]}
        />
      </LoopFrame>
      <div className="program-note">
        peek는 분기 최초 진입 1회만 실행합니다. E에서는 F 분기까지 확인한 뒤 E로 복귀하고, 5를 먼저 처리한 다음 F-6-7로 진행합니다.
      </div>
    </>
  );
}

function ReturnProgram({ on, tok }) {
  return (
    <LoopFrame label="RETURN" color={PALETTE.action}>
      <EvBlock color={PALETTE.start} name="Start" icon={Play} foot="MODE=1" width={56} />
      <Conn />
      <EvBlock color={PALETTE.data} name="Read Idx(rev)" icon={Upload} foot="i=LEN-1-k" active={on("READ")} width={92} />
      <Conn />
      <EvBlock color={PALETTE.data} name="Invert L/R" icon={ArrowLeftRight} foot={tok ? `-> ${TOKNAME[tok]}` : "1<->3"} active={on("READ")} width={82} />
      <Conn />
      <EvBlock color={PALETTE.my} name="Follow_Until_Event" icon={Route} foot="추종 유지" active={on("FOLLOW")} width={112} />
      <Conn />
      <EvBlock color={PALETTE.my} name="Turn_Until_Line" icon={Shuffle} foot="확신 회전" active={on("TURN")} width={92} />
    </LoopFrame>
  );
}

function PeekCase({ bits, label, active }) {
  return (
    <div className="peek-case" data-active={active ? "true" : "false"}>
      <b>{bits}</b>
      <span>{label}</span>
    </div>
  );
}
