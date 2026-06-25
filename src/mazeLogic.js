import { EDGES, EXPECTED_EXPLORE_PATH, NODES, NUMERIC_NODES } from "./mazeData.js";

export const ACTION_LABEL = { L: "좌(L)", S: "직(S)", R: "우(R)", B: "U턴", STOP: "정지" };
export const TOK = { L: 1, S: 2, R: 3, B: 4 };
export const TOKNAME = { 1: "L", 2: "S", 3: "R", 4: "U" };
export const THRESHOLD = 40;

const JUNCTIONS = ["A", "B", "D", "E", "F"];
const DIRS = ["L", "S", "R"];
const DIR_LABEL = { L: "좌", S: "직", R: "우", B: "U턴", STOP: "정지" };
const DIR_PRIORITY = { L: 1, S: 2, R: 3 };

export const ADJ = Object.fromEntries(Object.keys(NODES).map((id) => [id, []]));
EDGES.forEach(([a, b]) => {
  ADJ[a].push(b);
  ADJ[b].push(a);
});

export function edgePts(u, v) {
  for (const [a, b, pts] of EDGES) {
    if (a === u && b === v) return pts;
    if (a === v && b === u) return pts.slice().reverse();
  }
  return null;
}

export function angle(a, b) {
  return Math.atan2(b[1] - a[1], b[0] - a[0]);
}

export function angleDiff(a, b) {
  let d = b - a;
  while (d > Math.PI) d -= 2 * Math.PI;
  while (d < -Math.PI) d += 2 * Math.PI;
  return d;
}

export function lerpAngle(a, b, t) {
  return a + angleDiff(a, b) * t;
}

function startHeading(u, v) {
  const pts = edgePts(u, v);
  return angle(pts[0], pts[1]);
}

function endHeading(u, v) {
  const pts = edgePts(u, v);
  return angle(pts.at(-2), pts.at(-1));
}

function bfsPath(start, end) {
  const prev = { [start]: null };
  const q = [start];

  while (q.length) {
    const u = q.shift();
    if (u === end) break;
    for (const v of ADJ[u]) {
      if (!(v in prev)) {
        prev[v] = u;
        q.push(v);
      }
    }
  }

  const path = [];
  let cur = end;
  while (cur != null) {
    path.unshift(cur);
    cur = prev[cur];
  }
  return path;
}

function leftKids(cur, parent, incomingHeading) {
  return ADJ[cur]
    .filter((n) => n !== parent)
    .sort((a, b) => angleDiff(incomingHeading, startHeading(cur, a)) - angleDiff(incomingHeading, startHeading(cur, b)));
}

function closedDfs(node, parent, incomingHeading, out) {
  out.push(node);
  for (const child of leftKids(node, parent, incomingHeading)) {
    closedDfs(child, node, endHeading(node, child), out);
    out.push(node);
  }
}

function shouldProbeNextJunction(node, prev, next, arrival) {
  if (!next || NODES[next].kind !== "junction") return false;
  const exits = leftKids(node, prev, arrival);
  if (exits.length !== 2) return false;
  const hasLeafWork = exits.some((nb) => nb !== next && NODES[nb].kind === "leaf");
  return exits[0] === next && hasLeafWork;
}

export function coverEndAt(start, end) {
  const spine = bfsPath(start, end);
  const out = [];

  for (let i = 0; i < spine.length; i += 1) {
    const node = spine[i];
    const prev = spine[i - 1];
    const next = spine[i + 1];
    out.push(node);

    const arrival = prev ? endHeading(prev, node) : startHeading(node, next || ADJ[node][0]);
    if (shouldProbeNextJunction(node, prev, next, arrival)) {
      out.push(next);
      out.push(node);
    }

    const offshoots = ADJ[node]
      .filter((nb) => nb !== prev && nb !== next)
      .sort((a, b) => angleDiff(arrival, startHeading(node, a)) - angleDiff(arrival, startHeading(node, b)));

    for (const nb of offshoots) {
      closedDfs(nb, node, endHeading(node, nb), out);
      out.push(node);
    }
  }

  return out;
}

export function explorePath() {
  return coverEndAt("0", "7");
}

export function pathMatchesSpec(path) {
  return path.join(",") === EXPECTED_EXPLORE_PATH.join(",");
}

export function decideAt(seq, i) {
  const cur = seq[i];
  const prev = i > 0 ? seq[i - 1] : null;
  const next = i < seq.length - 1 ? seq[i + 1] : null;
  const incoming = prev != null ? endHeading(prev, cur) : next != null ? startHeading(cur, next) : 0;
  let L = 0;
  let C = 0;
  let R = 0;

  for (const nb of ADJ[cur]) {
    if (nb === prev) continue;
    const turn = angleDiff(incoming, startHeading(cur, nb));
    if (Math.abs(turn) > 2.5) continue;
    if (turn < -0.6) L = 1;
    else if (turn > 0.6) R = 1;
    else C = 1;
  }

  const node = NODES[cur];
  if (i === 0) return { node: cur, kind: "START", pattern: [L, C, R], action: "S", recorded: false };
  if (next === null) return { node: cur, kind: node.kind === "goal" ? "GOAL" : "END", pattern: [L, C, R], action: "STOP", recorded: false };

  const turn = angleDiff(incoming, startHeading(cur, next));
  const action = Math.abs(turn) > 2.5 ? "B" : turn < -0.6 ? "L" : turn > 0.6 ? "R" : "S";
  const kind = node.kind === "junction" ? "JCT" : "LEAF";
  return { node: cur, kind, pattern: [L, C, R], action, recorded: true };
}

export function buildLeg(seq) {
  const pts = [];
  const evt = {};

  seq.forEach((node, i) => {
    if (i === 0) {
      pts.push([NODES[node].x, NODES[node].y]);
      evt[0] = i;
      return;
    }

    const edge = edgePts(seq[i - 1], node);
    for (let k = 1; k < edge.length; k += 1) pts.push(edge[k]);
    evt[pts.length - 1] = i;
  });

  return { pts, evt, decisions: seq.map((_, i) => decideAt(seq, i)), seq };
}

export function invertToken(token) {
  if (token === 1) return 3;
  if (token === 3) return 1;
  return token;
}

function emptyMasks() {
  return Object.fromEntries(JUNCTIONS.map((id) => [id, { L: false, S: false, R: false }]));
}

function edgeKey(a, b) {
  return [a, b].sort().join("-");
}

function turnDir(incoming, from, to) {
  const turn = angleDiff(incoming, startHeading(from, to));
  if (Math.abs(turn) > 2.5) return "B";
  if (turn < -0.6) return "L";
  if (turn > 0.6) return "R";
  return "S";
}

function stackForNode(node) {
  if (!node || node === "0") return [];
  return bfsPath("0", node).filter((id) => NODES[id]?.kind === "junction");
}

function buildCandidateRows(seq, i, state) {
  const cur = seq[i];
  const prev = i > 0 ? seq[i - 1] : null;
  const next = i < seq.length - 1 ? seq[i + 1] : null;
  const incoming = prev != null ? endHeading(prev, cur) : next != null ? startHeading(cur, next) : 0;
  const byDir = Object.fromEntries(DIRS.map((dir) => [dir, null]));

  for (const nb of ADJ[cur] || []) {
    if (nb === prev) continue;
    const dir = turnDir(incoming, cur, nb);
    if (dir in byDir) byDir[dir] = nb;
  }

  return DIRS.map((dir) => {
    const node = byDir[dir];
    return {
      dir,
      label: DIR_LABEL[dir],
      node,
      open: Boolean(node),
      priority: node ? DIR_PRIORITY[dir] : null,
      done: node ? Boolean(state.exploredEdges?.[edgeKey(cur, node)]) : false,
      selected: Boolean(node && next === node),
    };
  });
}

function currentDecisionIndex(state) {
  const leg = state.legs[state.li];
  const max = leg.decisions.length - 1;
  return Math.max(0, Math.min(state.currentDecisionIndex ?? 0, max));
}

function decisionReason(state, decision, prev, next) {
  const mode = state.legs[state.li].mode;
  const pattern = decision.pattern.join("");

  if (mode === "RETURN") {
    const path = buildPathState(state);
    return `RETURN: path[${path.currentReadIndex}] ${path.sourceTokenName || "-"} -> 반전 ${path.invertedTokenName || "-"}; peek/scan 없이 기록대로 회전`;
  }

  if (decision.kind === "START") return "START: 중앙 라인을 잡고 Follow_Until_Event 진입";
  if (decision.kind === "GOAL") return "GOAL: 전 노드 통과 후 7 도착";
  if (decision.kind === "LEAF") return `${pattern}: 막다른 길 -> 색 확인 -> U턴 기록`;

  const seenBefore = state.currentDecisionSeenBefore ? 1 : 0;
  const peekType = state.peekTypes?.[decision.node];
  const selected = DIR_LABEL[decision.action] || decision.action;
  const nextText = next ? ` -> ${next}` : "";

  if (decision.pattern[0] === 1 && decision.pattern[2] === 1 && !state.currentDecisionSeenBefore) {
    const peekBits = peekType === "D" ? "010" : peekType === "B" ? "000" : "판독중";
    const typeText = peekType ? `${peekType}형` : "분기형 판독중";
    return `seen[${decision.node}]=${seenBefore}, ${pattern} -> peek -> ${peekBits} -> ${typeText}; ${selected} 선택${nextText}`;
  }

  if (decision.node === "E" && next === "F" && !state.eDiscoveryResolved) {
    return `seen[E]=${seenBefore}, ${pattern} -> 좌선우선 F 확인 -> 분기 발견 후 E 복귀 예약`;
  }

  return `seen[${decision.node}]=${seenBefore}, ${pattern} -> 미탐색 후보 중 좌>직>우 -> ${selected} 선택${nextText}`;
}

function buildDecisionDebug(state) {
  const leg = state.legs[state.li];
  const i = currentDecisionIndex(state);
  const decision = leg.decisions[i];
  const prev = i > 0 ? leg.seq[i - 1] : null;
  const next = i < leg.seq.length - 1 ? leg.seq[i + 1] : null;
  const patternNumber = decision.pattern.join("");
  return {
    node: decision.node,
    prev,
    next,
    kind: decision.kind,
    pattern: decision.pattern,
    patternNumber,
    action: decision.action,
    actionLabel: ACTION_LABEL[decision.action] || decision.action,
    recorded: decision.recorded,
    candidates: buildCandidateRows(leg.seq, i, state),
    reason: decisionReason(state, decision, prev, next),
  };
}

function buildMemoryDebug(state) {
  return {
    junctions: JUNCTIONS.map((id) => ({
      id,
      seen: Boolean(state.seenJ[id]),
      type: state.peekTypes[id] || "-",
      mask: state.junctionMasks[id] || { L: false, S: false, R: false },
    })),
    stack: stackForNode(buildDecisionDebug(state).node),
  };
}

function buildPathState(state) {
  const mode = state.legs[state.li].mode;
  const pathLen = mode === "EXPLORE" ? state.expRec : state.moveLog.length;
  const currentWriteIndex = mode === "EXPLORE" && state.expRec > 0 ? state.expRec - 1 : null;
  const currentReturnStep = mode === "RETURN" ? Math.max(0, Math.min(state.retRec - 1, state.moveLog.length - 1)) : null;
  const nextReturnStep = mode === "RETURN" ? Math.max(0, Math.min(state.retRec, state.moveLog.length - 1)) : null;
  const currentReadIndex = mode === "RETURN" ? state.moveLog.length - 1 - (currentReturnStep ?? 0) : null;
  const nextReadIndex = mode === "RETURN" ? state.moveLog.length - 1 - (nextReturnStep ?? 0) : null;
  const sourceToken = currentReadIndex == null ? null : state.moveLog[currentReadIndex];
  const invertedToken = sourceToken == null ? null : invertToken(sourceToken);

  return {
    path: state.moveLog,
    pathLen,
    currentWriteIndex,
    currentReadIndex,
    nextReadIndex,
    sourceToken,
    sourceTokenName: sourceToken ? TOKNAME[sourceToken] : null,
    invertedToken,
    invertedTokenName: invertedToken ? TOKNAME[invertedToken] : null,
  };
}

function selectedCandidate(decision) {
  return decision.candidates.find((row) => row.selected) || null;
}

function buildLesson(state, decision, memory, pathState) {
  const mode = state.legs[state.li].mode;
  const selected = selectedCandidate(decision);
  const selectedText = selected ? `${selected.label}쪽 ${selected.node}` : ACTION_LABEL[decision.action] || decision.action;
  const pattern = decision.patternNumber;
  const seenText = decision.kind === "JCT" ? `seen[${decision.node}]은 ${state.currentDecisionSeenBefore ? "1이라 재방문" : "0이라 첫 방문"}입니다.` : "";

  if (mode === "RETURN") {
    return {
      modeText: "복귀 중: 새 길을 찾지 않고, 탐사 때 저장한 회전 기록을 거꾸로 읽어 출발점으로 돌아갑니다.",
      headline: `path[${pathState.currentReadIndex}]의 ${pathState.sourceTokenName || "-"}를 ${pathState.invertedTokenName || "-"}로 바꿔 재생합니다.`,
      concept: "역재생",
      activeStep: 4,
      plainSteps: [
        "라인은 센서로 계속 따라가지만, 분기에서 새로 고민하지 않습니다.",
        `기록 배열을 뒤에서 읽습니다: path[${pathState.currentReadIndex}] = ${pathState.sourceTokenName || "-"}.`,
        "반대 방향으로 돌아가므로 좌/우만 바꿔 같은 길을 되짚습니다.",
      ],
      whyItMatters: "복귀 길은 좌표나 노드 이름을 외운 것이 아니라, 실제 탐사 중 쌓인 path[] 기록으로 결정됩니다.",
    };
  }

  if (decision.kind === "START") {
    return {
      modeText: "탐사 중: 로봇이 직접 센서를 읽고, 갈림길마다 판단하며 회전 기록을 쌓습니다.",
      headline: "출발점에서 중앙 센서가 검은 선을 잡고 라인 추종을 시작합니다.",
      concept: "라인 추종",
      activeStep: 0,
      plainSteps: [
        "중앙 센서가 선을 보고 있는지 계속 확인합니다.",
        "좌/우 센서는 평소에는 분기가 나타나는지 지켜봅니다.",
        "분기나 막다른 길이 나오면 잠시 멈추고 판단 단계로 넘어갑니다.",
      ],
      whyItMatters: "처음부터 정해진 좌표로 움직이지 않고, 센서가 읽은 선을 따라 다음 이벤트를 찾습니다.",
    };
  }

  if (decision.kind === "LEAF") {
    return {
      modeText: "탐사 중: 로봇이 직접 센서를 읽고, 갈림길마다 판단하며 회전 기록을 쌓습니다.",
      headline: `${decision.node}은 막다른 길입니다. 색을 확인하고 U턴을 기록합니다.`,
      concept: "막다른 길과 U턴",
      activeStep: 2,
      plainSteps: [
        `세 센서 패턴이 ${pattern}이라 더 갈 선이 없습니다.`,
        "이 지점을 방문했다고 표시한 뒤, 부모 분기로 돌아가기 위해 U턴합니다.",
        `방금 한 U턴을 path[${pathState.currentWriteIndex ?? state.expRec}]에 4로 기록합니다.`,
      ],
      whyItMatters: "막다른 길도 미리 적어 둔 순서가 아니라 센서 패턴 000을 보고 판단합니다.",
    };
  }

  if (decision.kind === "GOAL") {
    return {
      modeText: "탐사 중: 로봇이 직접 센서를 읽고, 갈림길마다 판단하며 회전 기록을 쌓습니다.",
      headline: "도착 7에 왔습니다. 모든 숫자 노드를 지난 뒤에야 탐사가 끝납니다.",
      concept: "모든 노드 방문",
      activeStep: 3,
      plainSteps: [
        "0부터 7까지 숫자 노드 방문 수를 확인합니다.",
        "중간 분기에서 남은 출구를 먼저 처리했기 때문에 7이 마지막에 나옵니다.",
        "이제 저장한 path[]를 사용해 RETURN 모드로 바뀝니다.",
      ],
      whyItMatters: "도착점으로 바로 달려간 것이 아니라, 남은 출구 기록과 방문 상태를 처리한 결과 7에 도착합니다.",
    };
  }

  if (state.micro === "PEEK" || (decision.pattern[0] === 1 && decision.pattern[2] === 1 && !state.currentDecisionSeenBefore)) {
    const type = state.peekTypes[decision.node];
    const result = type === "D" ? "가운데도 선이 보여 십자(D형)입니다." : type === "B" ? "가운데가 막혀 T자(B형)입니다." : "아직 가운데 센서를 확인하는 중입니다.";
    return {
      modeText: "탐사 중: 로봇이 직접 센서를 읽고, 갈림길마다 판단하며 회전 기록을 쌓습니다.",
      headline: `${decision.node}에서 좌우 선이 동시에 보여서 살짝 앞으로 확인합니다.`,
      concept: "peek와 seen",
      activeStep: 1,
      plainSteps: [
        `패턴 ${pattern}은 좌우가 모두 보이는 분기라 모양이 헷갈립니다.`,
        `${seenText} 그래서 최초 1회만 살짝 직진해 가운데 선을 확인합니다.`,
        `${result} 그 결과로 다음 회전 선택이 정해집니다.`,
      ],
      whyItMatters: "B인지 D인지 노드 이름으로 맞히지 않고, 실제 중앙 센서 확인 결과와 seen[] 기록으로 결정합니다.",
    };
  }

  if (state.micro === "DISCOVER" || (decision.node === "E" && decision.next === "F" && !state.eDiscoveryResolved)) {
    return {
      modeText: "탐사 중: 로봇이 직접 센서를 읽고, 갈림길마다 판단하며 회전 기록을 쌓습니다.",
      headline: "E에서 F가 또 다른 분기임을 확인하고, E의 남은 길을 먼저 끝냅니다.",
      concept: "한 분기 우선",
      activeStep: 1,
      plainSteps: [
        "좌선우선으로 보면 F 방향이 먼저라 실제로 가서 확인합니다.",
        "F가 새 분기라면 깊게 내려가지 않고 E로 돌아옵니다.",
        "E의 남은 잎 노드 5를 먼저 방문한 뒤, 다시 F 방향으로 진행합니다.",
      ],
      whyItMatters: "도착 7로 바로 가는 길을 숨겨 둔 것이 아니라, 분기 스택 흐름으로 현재 분기를 먼저 끝냅니다.",
    };
  }

  return {
    modeText: "탐사 중: 로봇이 직접 센서를 읽고, 갈림길마다 판단하며 회전 기록을 쌓습니다.",
    headline: `${decision.node} 분기에서 ${selectedText}을 선택합니다.`,
    concept: "좌선우선",
    activeStep: 1,
    plainSteps: [
      `세 센서 패턴 ${pattern}으로 열려 있는 방향을 찾습니다.`,
      "아직 가 보지 않은 방향 중 좌 -> 직진 -> 우 순서로 먼저 나오는 길을 고릅니다.",
      `선택한 회전은 path[]에 숫자로 기록되어 복귀 때 다시 쓰입니다.`,
    ],
    whyItMatters: "다음 노드를 직접 지정한 것이 아니라, 센서 패턴과 이미 가 본 방향 표를 보고 선택합니다.",
  };
}

export function buildDebugSnapshot(state) {
  if (!state) return null;
  const decision = buildDecisionDebug(state);
  const memory = buildMemoryDebug(state);
  const pathState = buildPathState(state);
  return {
    threshold: THRESHOLD,
    decision,
    memory,
    pathState,
    lesson: buildLesson(state, decision, memory, pathState),
  };
}

export function createSimulation() {
  const exp = explorePath();
  const ret = exp.slice().reverse();
  const exploreLeg = buildLeg(exp);
  const returnLeg = buildLeg(ret);
  const moveLog = exploreLeg.decisions.filter((d) => d.recorded).map((d) => TOK[d.action]);
  const revInv = moveLog.slice().reverse().map(invertToken);
  const start = exploreLeg.pts[0];

  return {
    specPathOk: pathMatchesSpec(exp),
    legs: [
      { ...exploreLeg, mode: "EXPLORE", base: 250 },
      { ...returnLeg, mode: "RETURN", base: 430 },
    ],
    li: 0,
    vi: 0,
    t: 0,
    phase: "move",
    x: start[0],
    y: start[1],
    heading: angle(exploreLeg.pts[0], exploreLeg.pts[1]),
    turnFrom: 0,
    turnTo: 0,
    turnT: 0,
    turnDur: 0.4,
    scanT: 0,
    scanWig: 0,
    pendVi: 0,
    turnNeeded: false,
    obsT: 0,
    time: 0,
    micro: "FOLLOW",
    curPat: [0, 1, 0],
    curAct: "S",
    peekT: 0,
    peekDir: 1,
    peekHeading: 0,
    peekClassified: false,
    peekDtype: false,
    seenJ: {},
    peekTypes: {},
    junctionMasks: emptyMasks(),
    exploredEdges: {},
    eDiscovered: false,
    eDiscoveryResolved: false,
    discT: 0,
    discLogged: false,
    visited: { "0": true },
    collected: { "0": true },
    moveLog,
    revInv,
    expRec: 0,
    retRec: 0,
    scoreOut: 0,
    scoreBack: 0,
    curTok: 0,
    bits: [0, 1, 0],
    refl: { L: 100, C: 10, R: 100 },
    currentDecisionIndex: 0,
    currentDecisionSeenBefore: false,
    log: [{ t: 0, m: "Start: 단일패스 좌선우선 + 111 peek, 전 노드 후 7" }],
    done: false,
    lastTs: 0,
  };
}

export function trimLog(state) {
  if (state.log.length > 16) state.log.pop();
}

export function fireNode(decisionIndex, state) {
  const leg = state.legs[state.li];
  const decision = leg.decisions[decisionIndex];
  const id = decision.node;
  const next = decisionIndex < leg.seq.length - 1 ? leg.seq[decisionIndex + 1] : null;
  state.currentDecisionIndex = decisionIndex;
  state.currentDecisionSeenBefore = Boolean(state.seenJ[id]);
  state.visited[id] = true;
  state.curPat = decision.pattern;
  state.curAct = decision.action;
  const countVisited = () => Object.keys(state.visited).filter((node) => NUMERIC_NODES.includes(node)).length;

  if (leg.mode === "EXPLORE") {
    if (decision.kind === "LEAF" || decision.kind === "GOAL") state.collected[id] = true;
    if (decision.recorded && next) state.exploredEdges[edgeKey(id, next)] = true;
    if (decision.kind === "JCT" && decision.action in DIR_PRIORITY) state.junctionMasks[id][decision.action] = true;

    if (decision.kind === "GOAL") {
      state.scoreOut = countVisited();
      state.log.unshift({ t: state.time, m: `도착 7: 직전 방문 ${state.scoreOut}/8, 전 노드 통과 후 종료` });
    } else if (decision.recorded) {
      state.curTok = TOK[decision.action];
      state.expRec += 1;
      state.log.unshift({
        t: state.time,
        m: `${decision.kind}: 패턴 ${decision.pattern.join("")} -> ${ACTION_LABEL[decision.action]} / LOG+=${TOKNAME[TOK[decision.action]]}`,
      });
    }
  } else if (id === "0") {
    state.scoreBack = countVisited();
    state.log.unshift({ t: state.time, m: `출발 0 복귀: 직전 방문 ${state.scoreBack}/8, 왕복 완료` });
  } else if (decision.recorded) {
    state.curTok = invertToken(TOK[leg.decisions[decisionIndex].action]);
    state.retRec += 1;
    state.log.unshift({ t: state.time, m: `복귀: 역재생 -> ${ACTION_LABEL[decision.action]}, 스캔 없이 확신 회전` });
  }

  trimLog(state);
}

export function uiSnapshot(state) {
  if (!state) return null;
  const leg = state.legs[state.li];
  return {
    mode: leg.mode,
    micro: state.micro,
    pat: state.curPat,
    act: state.curAct,
    tok: state.curTok,
    bits: state.bits,
    refl: state.refl,
    time: state.time,
    visited: Object.keys(state.visited),
    collected: Object.keys(state.collected).filter((node) => NUMERIC_NODES.includes(node)).length,
    moveLog: state.moveLog,
    expRec: state.expRec,
    revInv: state.revInv,
    retRec: state.retRec,
    log: state.log.slice(),
    done: state.done,
    scoreOut: state.scoreOut,
    scoreBack: state.scoreBack,
    specPathOk: state.specPathOk,
    debug: buildDebugSnapshot(state),
    peek: {
      active: state.micro === "PEEK",
      type: state.peekClassified ? (state.peekDtype ? "D" : "B") : null,
    },
  };
}
