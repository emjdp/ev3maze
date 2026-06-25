import { EDGES, EXPECTED_EXPLORE_PATH, NODES, NUMERIC_NODES } from "./mazeData.js";

export const ACTION_LABEL = { L: "좌(L)", S: "직(S)", R: "우(R)", B: "U턴", STOP: "정지" };
export const TOK = { L: 1, S: 2, R: 3, B: 4 };
export const TOKNAME = { 1: "L", 2: "S", 3: "R", 4: "U" };

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

export function coverEndAt(start, end) {
  const spine = bfsPath(start, end);
  const out = [];

  for (let i = 0; i < spine.length; i += 1) {
    const node = spine[i];
    const prev = spine[i - 1];
    const next = spine[i + 1];
    out.push(node);

    const arrival = prev ? endHeading(prev, node) : startHeading(node, next || ADJ[node][0]);
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
  const path = coverEndAt("0", "7");
  const eIndex = path.indexOf("E");
  if (eIndex >= 0) {
    path.splice(eIndex + 1, 0, "F", "E");
  }
  return path;
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
    eDiscovered: false,
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
  state.visited[id] = true;
  state.curPat = decision.pattern;
  state.curAct = decision.action;
  const countVisited = () => Object.keys(state.visited).filter((node) => NUMERIC_NODES.includes(node)).length;

  if (leg.mode === "EXPLORE") {
    if (decision.kind === "LEAF" || decision.kind === "GOAL") state.collected[id] = true;

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
    peek: {
      active: state.micro === "PEEK",
      type: state.peekClassified ? (state.peekDtype ? "D" : "B") : null,
    },
  };
}
