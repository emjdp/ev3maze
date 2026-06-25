import React, { useState, useRef, useEffect, useCallback } from "react";

/* ───────────────────────── 미로 데이터 ───────────────────────── */
const NODES = {
  "0": { x: 1010, y: 1125, kind: "start", color: "#F4C20D", label: "0" },
  "1": { x: 148, y: 1110, kind: "leaf", color: "#1C5FE0", label: "1" },
  "2": { x: 148, y: 420, kind: "leaf", color: "#1C5FE0", label: "2" },
  "3": { x: 648, y: 895, kind: "leaf", color: "#1C5FE0", label: "3" },
  "4": { x: 1140, y: 400, kind: "leaf", color: "#1C5FE0", label: "4" },
  "5": { x: 1108, y: 138, kind: "leaf", color: "#1C5FE0", label: "5" },
  "6": { x: 415, y: 395, kind: "leaf", color: "#1C5FE0", label: "6" },
  "7": { x: 205, y: 138, kind: "goal", color: "#1C5FE0", label: "7" },
  A: { x: 388, y: 870, kind: "junction" }, B: { x: 388, y: 658, kind: "junction" },
  D: { x: 868, y: 658, kind: "junction" }, E: { x: 868, y: 395, kind: "junction" },
  F: { x: 415, y: 138, kind: "junction" },
};
const EDGES = [
  ["7", "F", [[205, 138], [415, 138]]], ["F", "6", [[415, 138], [415, 395]]],
  ["F", "E", [[415, 138], [620, 138], [620, 395], [868, 395]]],
  ["5", "E", [[1108, 138], [868, 138], [868, 395]]], ["E", "D", [[868, 395], [868, 658]]],
  ["D", "4", [[868, 658], [1140, 658], [1140, 400]]], ["D", "3", [[868, 658], [868, 895], [648, 895]]],
  ["D", "B", [[868, 658], [388, 658]]], ["B", "2", [[388, 658], [148, 658], [148, 420]]],
  ["B", "A", [[388, 658], [388, 870]]], ["A", "1", [[388, 870], [148, 870], [148, 1110]]],
  ["A", "0", [[388, 870], [388, 1125], [1010, 1125]]],
];
const ADJ = {}; Object.keys(NODES).forEach((n) => (ADJ[n] = []));
EDGES.forEach(([a, b]) => { ADJ[a].push(b); ADJ[b].push(a); });
function edgePts(u, v) { for (const [a, b, p] of EDGES) { if (a === u && b === v) return p; if (a === v && b === u) return p.slice().reverse(); } return null; }
const ang = (a, b) => Math.atan2(b[1] - a[1], b[0] - a[0]);
const startH = (u, v) => { const p = edgePts(u, v); return ang(p[0], p[1]); };
const endH = (u, v) => { const p = edgePts(u, v); return ang(p[p.length - 2], p[p.length - 1]); };
function angDiff(a, b) { let d = b - a; while (d > Math.PI) d -= 2 * Math.PI; while (d < -Math.PI) d += 2 * Math.PI; return d; }
const lerpAng = (a, b, t) => a + angDiff(a, b) * t;

/* 좌선우선 완전 DFS (출발 0이 잎 → 전 노드 돌고 0 복귀) */
function leftHandWalk() {
  const seq = [];
  function dfs(cur, parent, hin) {
    seq.push(cur);
    let kids = ADJ[cur].filter((n) => n !== parent);
    kids.sort((a, b) => angDiff(hin, startH(cur, a)) - angDiff(hin, startH(cur, b)));
    for (const k of kids) { dfs(k, cur, endH(cur, k)); seq.push(cur); }
  }
  dfs("0", null, startH("0", ADJ["0"][0]));
  return seq;
}
/* 도착(7) 가지 지연 = 전 노드 방문 후 end에서 종료 (좌선 순서) */
function bfsPath(s, t) {
  const prev = { [s]: null }, q = [s];
  while (q.length) { const u = q.shift(); if (u === t) break; for (const v of ADJ[u]) if (!(v in prev)) { prev[v] = u; q.push(v); } }
  const p = []; let c = t; while (c != null) { p.unshift(c); c = prev[c]; } return p;
}
function leftKids(cur, parent, hin) { return ADJ[cur].filter((n) => n !== parent).sort((a, b) => angDiff(hin, startH(cur, a)) - angDiff(hin, startH(cur, b))); }
function closedDFS(u, parent, hin, out) { out.push(u); for (const v of leftKids(u, parent, hin)) { closedDFS(v, u, endH(u, v), out); out.push(u); } }
function coverEndAt(s, end) {
  const path = bfsPath(s, end), res = [];
  for (let i = 0; i < path.length; i++) {
    const node = path[i], prev = path[i - 1], next = path[i + 1];
    res.push(node);
    const arrH = prev ? endH(prev, node) : startH(node, next || ADJ[node][0]);
    const offs = ADJ[node].filter((nb) => nb !== prev && nb !== next).sort((a, b) => angDiff(arrH, startH(node, a)) - angDiff(arrH, startH(node, b)));
    for (const nb of offs) { closedDFS(nb, node, endH(node, nb), res); res.push(node); }
  }
  return res;
}
const ACT_KR = { L: "좌(L)", S: "직(S)", R: "우(R)", B: "U턴" };
const TOK = { L: 1, S: 2, R: 3, B: 4 };
const TOKNAME = { 1: "L", 2: "S", 3: "R", 4: "U" };
const invTok = (t) => (t === 1 ? 3 : t === 3 ? 1 : t);
function decideAt(seq, i) {
  const cur = seq[i], prev = i > 0 ? seq[i - 1] : null, next = i < seq.length - 1 ? seq[i + 1] : null;
  const hin = prev != null ? endH(prev, cur) : next != null ? startH(cur, next) : 0;
  let L = 0, C = 0, R = 0;
  for (const nb of ADJ[cur]) { if (nb === prev) continue; const t = angDiff(hin, startH(cur, nb)); if (Math.abs(t) > 2.5) continue; if (t < -0.6) L = 1; else if (t > 0.6) R = 1; else C = 1; }
  const n = NODES[cur]; let kind, action, recorded = false;
  if (i === 0) { kind = "START"; action = "S"; }
  else if (next === null) { kind = n.kind === "goal" ? "GOAL" : "END"; action = "STOP"; }
  else { const t = angDiff(hin, startH(cur, next)); action = Math.abs(t) > 2.5 ? "B" : t < -0.6 ? "L" : t > 0.6 ? "R" : "S"; kind = n.kind === "junction" ? "JCT" : "LEAF"; recorded = true; }
  return { node: cur, kind, pattern: [L, C, R], action, recorded };
}

/* 좌표 변환 */
const SCALE = 0.42, MARGIN = 40;
let MINX = 1e9, MINY = 1e9, MAXX = -1e9, MAXY = -1e9;
EDGES.forEach(([, , p]) => p.forEach(([x, y]) => { MINX = Math.min(MINX, x); MINY = Math.min(MINY, y); MAXX = Math.max(MAXX, x); MAXY = Math.max(MAXY, y); }));
const CW = (MAXX - MINX) * SCALE + MARGIN * 2, CH = (MAXY - MINY) * SCALE + MARGIN * 2;
const tx = (x) => (x - MINX) * SCALE + MARGIN, ty = (y) => (y - MINY) * SCALE + MARGIN;
const SENSOR_CFG = { B: { L: { f: 44, l: 38 }, C: { f: 44, l: 0 }, R: { f: 44, l: -38 } }, C: { L: { f: 30, l: 38 }, C: { f: 62, l: 0 }, R: { f: 30, l: -38 } } };

const COL = { bg: "#0e1116", panel: "#171c24", panel2: "#1e2530", line: "#2b3441", ink: "#e7eef5", dim: "#8893a2", surface: "#f4f3ee" };
const PAL = { flow: "#F6A623", sensor: "#F2C200", data: "#D0021B", action: "#6FBF1A", my: "#00A39C", start: "#5BB12F" };

function classify(r, g, b) {
  const lum = Math.round(((0.299 * r + 0.587 * g + 0.114 * b) / 255) * 100);
  let name = "WHITE";
  if (r > 200 && g > 160 && b < 120) name = "YEL"; else if (b > 150 && r < 140) name = "BLU"; else if (lum < 35) name = "BLK";
  return { refl: lum, name, bit: lum < 40 ? 1 : 0 };
}
function buildLeg(seq) {
  const pts = [], evt = {};
  seq.forEach((node, i) => { if (i === 0) { pts.push([NODES[node].x, NODES[node].y]); evt[0] = i; } else { const ep = edgePts(seq[i - 1], node); for (let k = 1; k < ep.length; k++) pts.push(ep[k]); evt[pts.length - 1] = i; } });
  return { pts, evt, decisions: seq.map((_, i) => decideAt(seq, i)), seq };
}

export default function EV3SimV5() {
  const canvasRef = useRef(null), offRef = useRef(null), simRef = useRef(null), rafRef = useRef(null);
  const runningRef = useRef(false), speedRef = useRef(1), cfgRef = useRef("B"), obsRef = useRef(false);
  const [running, setRunning] = useState(false), [cfg, setCfg] = useState("B"), [speed, setSpeed] = useState(1);
  const [ui, setUi] = useState(null);

  const renderMaze = useCallback(() => {
    const off = document.createElement("canvas"); off.width = Math.ceil(CW); off.height = Math.ceil(CH);
    const c = off.getContext("2d"); c.fillStyle = COL.surface; c.fillRect(0, 0, off.width, off.height);
    c.strokeStyle = "#15181c"; c.lineWidth = 13; c.lineCap = "round"; c.lineJoin = "round";
    EDGES.forEach(([, , p]) => { c.beginPath(); p.forEach(([x, y], i) => (i ? c.lineTo(tx(x), ty(y)) : c.moveTo(tx(x), ty(y)))); c.stroke(); });
    Object.entries(NODES).forEach(([id, n]) => { if (n.kind === "junction") return; const r = n.kind === "start" || n.kind === "goal" ? 15 : 13; c.beginPath(); c.arc(tx(n.x), ty(n.y), r, 0, 7); c.fillStyle = n.color; c.fill(); if (n.kind === "goal") { c.lineWidth = 3.5; c.strokeStyle = "#2bd47d"; c.stroke(); } });
    offRef.current = { off, data: c.getImageData(0, 0, off.width, off.height).data, w: off.width, h: off.height };
  }, []);
  const sample = useCallback((mx, my) => { const o = offRef.current; if (!o) return classify(244, 243, 238); const px = Math.round(tx(mx)), py = Math.round(ty(my)); if (px < 0 || py < 0 || px >= o.w || py >= o.h) return classify(244, 243, 238); const i = (py * o.w + px) * 4; return classify(o.data[i], o.data[i + 1], o.data[i + 2]); }, []);

  const resetSim = useCallback(() => {
    const exp = coverEndAt("0", "7");
    const ei = exp.indexOf("E"); if (ei >= 0) exp.splice(ei + 1, 0, "F", "E"); // E에서 좌선우선 → F까지 실제로 가봤다가 유턴해 복귀
    const ret = exp.slice().reverse();
    const legE = buildLeg(exp), legR = buildLeg(ret);
    const moveLog = legE.decisions.filter((d) => d.recorded).map((d) => TOK[d.action]);
    const revInv = moveLog.slice().reverse().map(invTok);
    const s = legE.pts[0];
    simRef.current = {
      legs: [{ ...legE, mode: "EXPLORE", base: 250 }, { ...legR, mode: "RETURN", base: 430 }],
      li: 0, vi: 0, t: 0, phase: "move", x: s[0], y: s[1], heading: ang(legE.pts[0], legE.pts[1]),
      turnFrom: 0, turnTo: 0, turnT: 0, turnDur: 0.4, scanT: 0, scanWig: 0, pendVi: 0, turnNeeded: false,
      obsT: 0, time: 0, micro: "FOLLOW", curPat: [0, 1, 0], curAct: "S",
      peekT: 0, peekDir: 1, peekHeading: 0, peekClassified: false, peekDtype: false,
      seenJ: {}, eDiscovered: false, discT: 0, discDir: 1, discHeading: 0, discLogged: false,
      visited: { "0": true }, collected: { "0": true }, moveLog, revInv, expRec: 0, retRec: 0,
      scoreOut: 0, scoreBack: 0,
      curTok: 0, log: [{ t: 0, m: "Start · 단일패스: 좌선우선 + 111 peek → 전 노드 후 7" }], done: false, lastTs: 0,
    };
    pushUi(); drawFrame(0);
  }, []);

  const NUM = ["0", "1", "2", "3", "4", "5", "6", "7"];
  function fireNode(di, S) {
    const leg = S.legs[S.li], d = leg.decisions[di], id = d.node;
    S.visited[id] = true; S.curPat = d.pattern; S.curAct = d.action;
    const cnt = () => Object.keys(S.visited).filter((n) => NUM.includes(n)).length;
    if (leg.mode === "EXPLORE") {
      if (d.kind === "LEAF" || d.kind === "GOAL") { S.collected[id] = true; }
      if (d.kind === "GOAL") { S.scoreOut = cnt(); S.log.unshift({ t: S.time, m: `★ 도착 7 — 직전 방문 ${S.scoreOut}/8 (전 노드 통과 후 7)` }); }
      else if (d.recorded) { S.curTok = TOK[d.action]; S.expRec++; S.log.unshift({ t: S.time, m: `${d.kind} · 패턴 ${d.pattern.join("")} → ${ACT_KR[d.action]} · LOG+=${TOKNAME[TOK[d.action]]}` }); }
    } else {
      if (id === "0") { S.scoreBack = cnt(); S.log.unshift({ t: S.time, m: `★ 출발 0 복귀 — 직전 방문 ${S.scoreBack}/8 · 왕복 완료` }); }
      else if (d.recorded) { S.curTok = invTok(TOK[leg.decisions[di].action]); S.retRec++; S.log.unshift({ t: S.time, m: `복귀 · 역재생 → ${ACT_KR[d.action]} (스캔 없이 확신 회전)` }); }
    }
    if (S.log.length > 16) S.log.pop();
  }

  function pushUi() {
    const S = simRef.current; if (!S) return;
    const leg = S.legs[S.li];
    setUi({
      mode: leg.mode, micro: S.micro, pat: S.curPat, act: S.curAct, tok: S.curTok,
      bits: S.bits || [0, 1, 0], refl: S.refl || { L: 100, C: 10, R: 100 },
      time: S.time, visited: Object.keys(S.visited), collected: Object.keys(S.collected).filter((n) => NUM.includes(n)).length,
      moveLog: S.moveLog, expRec: S.expRec, revInv: S.revInv, retRec: S.retRec, log: S.log.slice(), done: S.done,
      scoreOut: S.scoreOut, scoreBack: S.scoreBack,
      peek: { active: S.micro === "PEEK", type: S.peekClassified ? (S.peekDtype ? "D" : "B") : null },
    });
  }

  const drawFrame = useCallback((dtEff) => {
    const S = simRef.current; if (!S) return;
    const leg = S.legs[S.li], pts = leg.pts, speedU = leg.base;

    if (!S.done && dtEff > 0) {
      S.time += dtEff;
      if (S.phase === "obstacle") {
        S.obsT -= dtEff; S.micro = "OBSTACLE";
        if (S.obsT <= 0) { S.phase = "move"; S.micro = "FOLLOW"; S.log.unshift({ t: S.time, m: "장애물 처리 완료 → 추종 재개" }); if (S.log.length > 16) S.log.pop(); }
      } else if (S.phase === "move") {
        if (obsRef.current && leg.mode === "EXPLORE") { obsRef.current = false; S.phase = "obstacle"; S.obsT = 1.6; S.micro = "OBSTACLE"; S.log.unshift({ t: S.time, m: "초음파 감지 → Loop Interrupt LF → Handle_Obstacle(리프트)" }); }
        else {
          const a = pts[S.vi], b = pts[S.vi + 1], len = Math.hypot(b[0] - a[0], b[1] - a[1]);
          S.t += (speedU * dtEff) / len; S.micro = "FOLLOW"; S.curPat = [0, 1, 0]; S.curAct = "S";
          if (S.t >= 1) {
            S.t = 1; S.x = b[0]; S.y = b[1]; const arr = S.vi + 1; const isNode = arr in leg.evt;
            if (isNode) { fireNode(leg.evt[arr], S); S.micro = "READ"; }
            if (arr >= pts.length - 1) {
              if (S.li < S.legs.length - 1) { S.li += 1; S.vi = 0; S.t = 0; S.phase = "move"; const np = S.legs[S.li].pts; S.x = np[0][0]; S.y = np[0][1]; S.heading = ang(np[0], np[1]); S.log.unshift({ t: S.time, m: "탐사 완료 · MODE=RETURN · MOVELOG 역재생 시작" }); }
              else { S.done = true; S.phase = "done"; S.micro = "DONE"; runningRef.current = false; setRunning(false); }
            } else {
              const inA = ang(pts[S.vi], pts[arr]), outA = ang(pts[arr], pts[arr + 1]), turn = Math.abs(angDiff(inA, outA)) > 0.4;
              const dec = isNode ? leg.decisions[leg.evt[arr]] : null;
              const isJ = dec && (dec.kind === "JCT" || dec.kind === "LEAF");
              const seen = dec && S.seenJ[dec.node];
              const needPeek = dec && dec.pattern[0] === 1 && dec.pattern[2] === 1 && !seen; // 좌·우 둘 다 + 최초 진입만
              const fDiscover = leg.mode === "EXPLORE" && dec && dec.node === "F" && leg.seq[arr + 1] === "E" && !S.eDiscovered; // F까지 가봤다가 유턴
              if (dec && dec.kind === "JCT") S.seenJ[dec.node] = true; // 분기 메모리: 방문 기록
              S.turnFrom = inA; S.turnTo = outA; S.turnNeeded = turn; S.pendVi = arr;
              if (fDiscover) { S.eDiscovered = true; S.phase = "discover"; S.discT = 0.55; S.discLogged = false; S.micro = "DISCOVER"; }
              else if (leg.mode === "EXPLORE" && needPeek) { S.phase = "peek"; S.peekT = 0; S.peekDir = 1; S.peekHeading = inA; S.peekClassified = false; S.micro = "PEEK"; }
              else if (leg.mode === "EXPLORE" && isJ) { S.phase = "scan"; S.scanT = 0.4; S.scanWig = 0; S.micro = "DECIDE"; }
              else if (turn) { S.phase = "turn"; S.turnT = 0; S.turnDur = Math.max(0.14, Math.abs(angDiff(inA, outA)) / Math.PI * 0.38); S.micro = "TURN"; }
              else { S.vi = arr; S.t = 0; }
            }
          } else { S.x = a[0] + (b[0] - a[0]) * S.t; S.y = a[1] + (b[1] - a[1]) * S.t; S.heading = ang(a, b); }
        }
      } else if (S.phase === "scan") {
        S.scanT -= dtEff; S.scanWig += dtEff * 11; S.heading = S.turnFrom + Math.sin(S.scanWig) * 0.32; S.micro = "DECIDE";
        if (S.scanT <= 0) { if (S.turnNeeded) { S.phase = "turn"; S.turnT = 0; S.turnDur = Math.max(0.14, Math.abs(angDiff(S.turnFrom, S.turnTo)) / Math.PI * 0.38); S.micro = "TURN"; } else { S.vi = S.pendVi; S.t = 0; S.phase = "move"; S.heading = S.turnTo; } }
      } else if (S.phase === "peek") {
        const jp = pts[S.pendVi], hx = Math.cos(S.peekHeading), hy = Math.sin(S.peekHeading);
        S.peekT = Math.max(0, Math.min(1, S.peekT + dtEff * 2.4 * S.peekDir));
        S.x = jp[0] + hx * 42 * S.peekT; S.y = jp[1] + hy * 42 * S.peekT; S.heading = S.peekHeading; S.micro = "PEEK";
        if (S.peekDir > 0 && S.peekT >= 1) {
          if (!S.peekClassified) {
            S.peekClassified = true;
            const dec = leg.decisions[leg.evt[S.pendVi]];
            S.peekDtype = dec.pattern[1] === 1; // 직진 출구 있음 → D형(십자)
            S.log.unshift({ t: S.time, m: S.peekDtype ? "111 → peek 직진 → 010 → D형(십자): 이왕 온김에 직진" : "111 → peek 직진 → 000 → B형(T): 후진 후 좌선우선" });
            if (S.log.length > 16) S.log.pop();
          }
          if (S.peekDtype && !S.turnNeeded) { S.vi = S.pendVi; S.t = 0.28; S.phase = "move"; } // D형: 그대로 직진
          else S.peekDir = -1; // 후진
        } else if (S.peekDir < 0 && S.peekT <= 0) {
          if (S.turnNeeded) { S.phase = "turn"; S.turnT = 0; S.turnDur = Math.max(0.14, Math.abs(angDiff(S.turnFrom, S.turnTo)) / Math.PI * 0.38); S.micro = "TURN"; }
          else { S.vi = S.pendVi; S.t = 0; S.phase = "move"; S.heading = S.turnTo; }
        }
      } else if (S.phase === "discover") {
        const jp = pts[S.pendVi]; S.x = jp[0]; S.y = jp[1]; S.heading = S.turnFrom; S.micro = "DISCOVER";
        if (!S.discLogged) { S.discLogged = true; S.log.unshift({ t: S.time, m: "좌선우선으로 F까지 가봄 → F 분기 발견 → 유턴해 E로 → 5부터 (한 분기 끝나기 전 다른 분기 X)" }); if (S.log.length > 16) S.log.pop(); }
        S.discT -= dtEff;
        if (S.discT <= 0) { S.phase = "turn"; S.turnT = 0; S.turnDur = Math.max(0.16, Math.abs(angDiff(S.turnFrom, S.turnTo)) / Math.PI * 0.4); S.micro = "TURN"; }
      } else if (S.phase === "turn") {
        S.turnT += dtEff / S.turnDur; S.micro = "TURN";
        if (S.turnT >= 1) { S.heading = S.turnTo; S.vi = S.pendVi; S.t = 0; S.phase = "move"; } else S.heading = lerpAng(S.turnFrom, S.turnTo, S.turnT);
      }
    }

    // 센서
    const cfgO = SENSOR_CFG[cfgRef.current], ch = Math.cos(S.heading), sh = Math.sin(S.heading);
    const sw = (o) => [S.x + o.f * ch + o.l * -sh, S.y + o.f * sh + o.l * ch];
    const sL = sw(cfgO.L), sC = sw(cfgO.C), sR = sw(cfgO.R), rL = sample(...sL), rC = sample(...sC), rR = sample(...sR);
    S.bits = [rL.bit, rC.bit, rR.bit]; S.refl = { L: rL.refl, C: rC.refl, R: rR.refl };

    // 그리기
    const cv = canvasRef.current;
    if (cv && offRef.current) {
      const c = cv.getContext("2d"); c.clearRect(0, 0, CW, CH); c.drawImage(offRef.current.off, 0, 0);
      Object.keys(S.visited).forEach((id) => { const n = NODES[id]; if (n.kind === "junction") return; c.beginPath(); c.arc(tx(n.x), ty(n.y), 19, 0, 7); c.strokeStyle = "rgba(43,212,125,.6)"; c.lineWidth = 2.4; c.stroke(); });
      c.font = "bold 11px ui-monospace,monospace"; c.textAlign = "center"; c.textBaseline = "middle";
      Object.entries(NODES).forEach(([id, n]) => { if (n.kind === "junction") { c.fillStyle = COL.dim; c.fillText(id, tx(n.x) + 14, ty(n.y) - 14); } else { c.fillStyle = n.kind === "start" ? "#3a2d00" : "#fff"; c.fillText(n.label, tx(n.x), ty(n.y)); } });
      const rx = tx(S.x), ry = ty(S.y); c.save(); c.translate(rx, ry); c.rotate(S.heading);
      if (S.phase === "obstacle") { c.fillStyle = "#ff8a3d"; c.fillRect(20, -10, 14, 20); }
      c.fillStyle = "#2b3441"; c.strokeStyle = PAL.flow; c.lineWidth = 2; c.beginPath(); c.roundRect(-14, -11, 26, 22, 5); c.fill(); c.stroke();
      c.fillStyle = "#0b0e12"; c.fillRect(-8, -14, 10, 4); c.fillRect(-8, 10, 10, 4); c.restore();
      const ds = (p, rd) => { const col = rd.name === "BLU" ? "#3b82f6" : rd.name === "YEL" ? "#F4C20D" : rd.name === "BLK" ? "#111" : "#cfd6dd"; c.beginPath(); c.arc(tx(p[0]), ty(p[1]), 4.2, 0, 7); c.fillStyle = col; c.fill(); c.lineWidth = 1.4; c.strokeStyle = rd.name === "WHITE" ? "#94a0ad" : "#fff"; c.stroke(); };
      ds(sL, rL); ds(sR, rR); ds(sC, rC);
    }
    pushUi();
  }, [sample]);

  useEffect(() => {
    const loop = (ts) => { const S = simRef.current; if (!S) { rafRef.current = requestAnimationFrame(loop); return; } const last = S.lastTs || ts; let dt = (ts - last) / 1000; S.lastTs = ts; dt = Math.min(dt, 0.05); drawFrame(runningRef.current && !S.done ? dt * speedRef.current : 0); rafRef.current = requestAnimationFrame(loop); };
    renderMaze(); resetSim(); rafRef.current = requestAnimationFrame(loop); return () => cancelAnimationFrame(rafRef.current);
  }, [renderMaze, resetSim, drawFrame]);

  const onReset = () => { runningRef.current = false; setRunning(false); obsRef.current = false; if (simRef.current) simRef.current.lastTs = 0; resetSim(); };
  const onToggle = () => { const r = !running; runningRef.current = r; setRunning(r); if (simRef.current) simRef.current.lastTs = 0; };
  if (!ui) return <div style={{ background: COL.bg, color: COL.ink, padding: 20 }}>로딩…</div>;

  const m = ui.micro, mode = ui.mode;
  const on = (...s) => s.includes(m);
  const patStr = ui.pat.join("");

  return (
    <div style={{ background: COL.bg, color: COL.ink, padding: 16, borderRadius: 14, fontFamily: "system-ui,sans-serif" }}>
      <div style={{ display: "flex", justifyContent: "space-between", alignItems: "baseline", flexWrap: "wrap", gap: 8, marginBottom: 10 }}>
        <div>
          <div style={{ fontSize: 18, fontWeight: 800 }}>EV3 Lab 미로 시뮬레이터 <span style={{ color: PAL.flow }}>ver5</span></div>
          <div style={{ fontSize: 11, color: COL.dim, marginTop: 2 }}>단일패스 좌선우선 + 111 peek(B/D형 변별) · MOVELOG 역재생 복귀</div>
        </div>
        <div style={{ display: "flex", gap: 6 }}>
          <Pill on={mode === "EXPLORE"} c={PAL.flow}>EXPLORE</Pill>
          <Pill on={mode === "RETURN"} c={PAL.action}>RETURN</Pill>
          {ui.done && <Pill on c={PAL.action}>완료</Pill>}
        </div>
      </div>

      {/* ── EV3-G 실행 블록 다이어그램 ── */}
      <div style={{ background: "#fbfbf9", borderRadius: 10, padding: 14, border: `1px solid ${COL.line}`, overflowX: "auto" }}>
        <div style={{ fontSize: 10.5, color: "#7a8694", marginBottom: 8, fontWeight: 700, letterSpacing: .5 }}>
          EV3-G 프로그램 — {mode === "EXPLORE" ? "탐사 루프 (EXPLORE)" : "복귀 루프 (RETURN)"} · 실행 중 블록이 빛납니다
        </div>
        {mode === "EXPLORE" ? <ExploreProgram on={on} patStr={patStr} act={ui.act} peek={ui.peek} /> : <ReturnProgram on={on} tok={ui.tok} />}
        <div style={{ display: "flex", gap: 12, marginTop: 10, fontSize: 9.5, color: "#8893a2", flexWrap: "wrap" }}>
          <Lg c={PAL.flow}>Flow</Lg><Lg c={PAL.sensor}>Sensor</Lg><Lg c={PAL.data}>Data</Lg><Lg c={PAL.action}>Action</Lg><Lg c={PAL.my}>My Block</Lg>
        </div>
      </div>

      <div style={{ display: "flex", gap: 12, marginTop: 12, flexWrap: "wrap" }}>
        {/* 캔버스 */}
        <div style={{ background: COL.panel, borderRadius: 10, padding: 9, border: `1px solid ${COL.line}` }}>
          <canvas ref={canvasRef} width={Math.ceil(CW)} height={Math.ceil(CH)} style={{ display: "block", borderRadius: 6, maxWidth: "100%" }} />
        </div>

        <div style={{ flex: 1, minWidth: 280, display: "flex", flexDirection: "column", gap: 10 }}>
          {/* 채점 */}
          <Panel title="채점 (구간별 도달 직전 방문 노드)">
            <div style={{ display: "flex", gap: 8 }}>
              <ScoreBox label="출발 → 도착 7" v={ui.scoreOut} done={ui.scoreOut > 0} />
              <ScoreBox label="도착 → 출발 0" v={ui.scoreBack} done={ui.scoreBack > 0} />
            </div>
            <div style={{ fontSize: 10, color: COL.dim, marginTop: 6 }}>각 구간 끝점 도달 시 8/8 이면 만점 (전 노드 통과 후 종료)</div>
          </Panel>
          {/* 센서 */}
          <Panel title={`컬러센서 (배치 ${cfg}) — 패턴 ${ui.bits.join(" ")}`}>
            <div style={{ display: "flex", gap: 8 }}>
              {["L", "C", "R"].map((k, i) => { const rf = ui.refl[k], bit = ui.bits[i]; return (
                <div key={k} style={{ flex: 1, textAlign: "center" }}>
                  <div style={{ fontSize: 9.5, color: COL.dim }}>{k}</div>
                  <div style={{ height: 40, background: "#0b0e12", borderRadius: 4, position: "relative", overflow: "hidden", border: `1px solid ${COL.line}`, marginTop: 2 }}><div style={{ position: "absolute", bottom: 0, left: 0, right: 0, height: `${rf}%`, background: bit ? "#3a4350" : "#d7dde3" }} /></div>
                  <div style={{ fontFamily: "ui-monospace,monospace", fontSize: 15, fontWeight: 800, color: bit ? PAL.flow : COL.dim }}>{bit}</div>
                </div>); })}
            </div>
          </Panel>

          {/* 변수 */}
          <Panel title="변수 (MOVELOG)">
            <Row k="MODE" v={mode} c={mode === "EXPLORE" ? PAL.flow : PAL.action} />
            <Row k="micro" v={m} />
            <div style={{ fontSize: 11, color: COL.dim, marginTop: 6, marginBottom: 3 }}>path 기록 (탐사):</div>
            <TokenStrip arr={ui.moveLog} n={ui.expRec} active={mode === "EXPLORE"} />
            <div style={{ fontSize: 11, color: COL.dim, marginTop: 8, marginBottom: 3 }}>역재생 (복귀, 반전):</div>
            <TokenStrip arr={ui.revInv} n={ui.retRec} active={mode === "RETURN"} />
            <div style={{ display: "flex", gap: 4, marginTop: 8, flexWrap: "wrap" }}>
              {NUM.map((n) => { const v = ui.visited.includes(n); return <div key={n} style={{ width: 21, height: 21, borderRadius: 5, fontSize: 11, fontWeight: 700, display: "flex", alignItems: "center", justifyContent: "center", background: v ? (NODES[n].kind === "start" ? "#F4C20D" : NODES[n].kind === "goal" ? "#2bd47d" : "#3b82f6") : COL.panel2, color: v ? (NODES[n].kind === "start" ? "#3a2d00" : "#fff") : COL.dim, border: `1px solid ${v ? "transparent" : COL.line}` }}>{n}</div>; })}
            </div>
            <div style={{ fontSize: 10.5, color: COL.dim, marginTop: 6 }}>수집 {ui.collected}/8 · {ui.time.toFixed(1)}s</div>
          </Panel>

          <Panel title="실행 로그">
            <div style={{ maxHeight: 110, overflowY: "auto", display: "flex", flexDirection: "column", gap: 3 }}>
              {ui.log.map((l, i) => <div key={i} style={{ fontSize: 10.5, color: i === 0 ? COL.ink : COL.dim, fontFamily: "ui-monospace,monospace", display: "flex", gap: 6 }}><span style={{ color: PAL.flow, flexShrink: 0 }}>{l.t.toFixed(1)}</span><span>{l.m}</span></div>)}
            </div>
          </Panel>
        </div>
      </div>

      {/* 컨트롤 */}
      <div style={{ display: "flex", gap: 9, marginTop: 12, alignItems: "center", flexWrap: "wrap" }}>
        <button onClick={onToggle} style={btn(running ? "#ff5d5d" : PAL.action)}>{running ? "❚❚ 정지" : ui.done ? "↺ 재생" : "▶ 실행"}</button>
        <button onClick={onReset} style={btn(COL.panel2, COL.ink)}>↺ 리셋</button>
        <button onClick={() => { obsRef.current = true; }} style={btn("#ff8a3d")} disabled={mode !== "EXPLORE"}>⚠ 장애물 주입</button>
        <div style={{ display: "flex", gap: 4, background: COL.panel2, padding: 3, borderRadius: 8, border: `1px solid ${COL.line}` }}>
          {["B", "C"].map((x) => <button key={x} onClick={() => { cfgRef.current = x; setCfg(x); }} style={{ ...btn(cfg === x ? PAL.flow : "transparent", cfg === x ? "#1a1500" : COL.dim), padding: "5px 11px", fontSize: 12 }}>배치 {x}</button>)}
        </div>
        <div style={{ display: "flex", alignItems: "center", gap: 7, marginLeft: "auto" }}>
          <span style={{ fontSize: 11, color: COL.dim }}>×{speed.toFixed(1)}</span>
          <input type="range" min={0.3} max={3} step={0.1} value={speed} onChange={(e) => { const v = parseFloat(e.target.value); speedRef.current = v; setSpeed(v); }} style={{ width: 110, accentColor: PAL.flow }} />
        </div>
      </div>
    </div>
  );
}

/* ── EV3-G 블록 컴포넌트 ── */
function EvBlock({ color, name, icon, foot, active, w = 84 }) {
  return (
    <div style={{ width: w, borderRadius: 6, background: "#fff", border: `1px solid ${active ? "#1f8bff" : "#c7ccd2"}`, boxShadow: active ? "0 0 0 2px #1f8bff, 0 0 14px rgba(31,139,255,.5)" : "0 1px 2px rgba(0,0,0,.12)", transition: "all .1s", flexShrink: 0 }}>
      <div style={{ background: color, color: "#fff", fontSize: 9.5, fontWeight: 700, padding: "2px 6px", borderRadius: "5px 5px 0 0", whiteSpace: "nowrap", overflow: "hidden", textOverflow: "ellipsis" }}>{name}</div>
      <div style={{ padding: "8px 4px", textAlign: "center", fontSize: 17, minHeight: 22 }}>{icon}</div>
      <div style={{ background: "#eceff2", color: "#555", fontSize: 9, padding: "2px 5px", borderRadius: "0 0 5px 5px", whiteSpace: "nowrap", overflow: "hidden", textOverflow: "ellipsis" }}>{foot}</div>
    </div>
  );
}
const Conn = () => <div style={{ width: 10, height: 2, background: "#9aa3ad", flexShrink: 0 }} />;
function LoopFrame({ label, color, children }) {
  return (
    <div style={{ position: "relative", border: `3px solid ${color}`, borderRadius: 14, padding: "20px 14px 14px", marginTop: 8, minWidth: "fit-content" }}>
      <div style={{ position: "absolute", top: -11, left: "50%", transform: "translateX(-50%)", background: color, color: "#fff", fontSize: 10, fontWeight: 800, padding: "1px 12px", borderRadius: 5 }}>{label}</div>
      <div style={{ display: "flex", alignItems: "center", minWidth: "fit-content" }}>{children}</div>
      <div style={{ position: "absolute", right: 8, bottom: 4, fontSize: 12, color }}>↻ ∞</div>
    </div>
  );
}
function SwitchBlock({ cases, activeCase }) {
  return (
    <div style={{ border: "2px solid #b9c0c8", borderRadius: 8, background: "#f6f8fa", padding: 4, flexShrink: 0 }}>
      {cases.map((cs, i) => (
        <div key={i} style={{ display: "flex", alignItems: "center", padding: "4px 6px", borderRadius: 6, marginBottom: i < cases.length - 1 ? 4 : 0, background: activeCase === i ? "rgba(31,139,255,.12)" : "transparent", border: `1px solid ${activeCase === i ? "#1f8bff" : "transparent"}` }}>
          <div style={{ fontSize: 10, fontWeight: 800, color: activeCase === i ? "#1f8bff" : "#8893a2", width: 64, flexShrink: 0 }}>{activeCase === i ? "● " : "○ "}{cs.label}</div>
          <div style={{ display: "flex", alignItems: "center" }}>{cs.body}</div>
        </div>
      ))}
    </div>
  );
}

function ExploreProgram({ on, patStr, act, peek }) {
  const following = on("FOLLOW"), deciding = on("READ", "DECIDE", "TURN"), obstacle = on("OBSTACLE");
  const peekOn = on("PEEK"), pType = peek && peek.type;
  const discoverOn = on("DISCOVER");
  const rows = [
    ["010", "직진 S", "010"], ["100", "좌회전 L", "100"], ["110", "좌회전 L", "110"],
    ["101", "좌회전 L", "101"], ["111", "좌회전 L", "111"], ["011", "직진 S", "011"],
    ["001", "우회전 R", "001"], ["000", "U턴 · 색기록", "000"],
  ];
  return (
    <div>
    <LoopFrame label="MAIN  ⟳" color={PAL.flow}>
      <EvBlock color={PAL.start} name="Start" icon="▶" foot="" active={false} w={56} />
      <Conn />
      <EvBlock color={PAL.my} name="Follow_Until_Event" icon="🛣️" foot="LF loop" active={following} w={104} />
      <Conn />
      <SwitchBlock activeCase={obstacle ? 1 : 0} cases={[
        { label: "Event=0", body: (
          <div style={{ display: "flex", alignItems: "center" }}>
            <EvBlock color={PAL.my} name="Read_Pattern" icon="◐◐◐" foot={patStr} active={deciding || peekOn} w={80} />
            <Conn />
            {/* 111(좌·우) → peek 직진 → 000/010 분기 */}
            <div style={{ display: "flex", flexDirection: "column", gap: 4, marginRight: 8, padding: 5, borderRadius: 7, border: `1.5px dashed ${peekOn ? PAL.flow : "#cfd6dd"}`, background: peekOn ? "rgba(246,166,35,.08)" : "transparent" }}>
              <div style={{ fontSize: 9, color: "#8893a2", fontWeight: 700 }}>111(좌·우)? → Peek 직진</div>
              <EvBlock color={PAL.sensor} name="Peek 직진" icon="🔎" foot="C센서 판독" active={peekOn} w={78} />
              <div style={{ display: "flex", flexDirection: "column", gap: 2 }}>
                <PeekCase bits="010" lbl="D형(십자) → 직진 먼저" act={pType === "D"} />
                <PeekCase bits="000" lbl="B형(T) → 후진·좌선" act={pType === "B"} />
              </div>
            </div>
            <div style={{ display: "flex", flexDirection: "column", gap: 2, marginRight: 6 }}>
              {rows.map(([bits, lbl]) => { const a = deciding && patStr === bits; return (
                <div key={bits} style={{ display: "flex", alignItems: "center", gap: 6, fontSize: 9.5, padding: "1px 6px", borderRadius: 4, background: a ? "rgba(246,166,35,.25)" : "#fff", border: `1px solid ${a ? PAL.flow : "#dde2e7"}`, boxShadow: a ? `0 0 8px ${PAL.flow}66` : "none" }}>
                  <b style={{ fontFamily: "ui-monospace,monospace", letterSpacing: 1, color: a ? "#222" : "#8893a2", width: 26 }}>{bits}</b>
                  <span style={{ color: a ? "#c06a00" : "#8893a2", fontWeight: 700 }}>{lbl}</span>
                </div>); })}
            </div>
            <EvBlock color={PAL.my} name="Turn_Until_Line" icon="↻" foot={act} active={on("TURN")} w={86} />
            <Conn />
            <EvBlock color={PAL.flow} name="한 분기 우선" icon="⛔" foot="F까지→유턴" active={discoverOn} w={84} />
            <Conn />
            <EvBlock color={PAL.data} name="Array Write" icon="📥" foot="MOVELOG" active={on("READ")} w={72} />
          </div>
        ) },
        { label: "Event=1", body: (
          <EvBlock color={PAL.my} name="Handle_Obstacle" icon="🦾" foot="리프트(예약)" active={obstacle} w={96} />
        ) },
      ]} />
    </LoopFrame>
    <div style={{ fontSize: 10.5, color: "#8893a2", marginTop: 8, paddingLeft: 4 }}>
      ※ <b style={{ color: "#c06a00" }}>peek는 분기 최초 진입 1회만</b>(B 1회·D 1회=2회). 4·3에서 D로 되돌아오거나 A·F에선 이미 아는 분기라 peek 안 함.
      <b style={{ color: "#c06a00" }}> 한 분기 우선</b>: E에서 좌선우선으로 <b>F까지 실제로 가봄</b> → F 분기 발견 → 유턴해 E 복귀 → 5부터(다른 분기를 먼저 탐색하지 않음).
    </div>
    </div>
  );
}
function ReturnProgram({ on, tok }) {
  return (
    <LoopFrame label="RETURN  ⟳" color={PAL.action}>
      <EvBlock color={PAL.start} name="Start" icon="▶" foot="MODE=1" active={false} w={56} />
      <Conn />
      <EvBlock color={PAL.data} name="Read Idx(rev)" icon="📤" foot="i=LEN-1-k" active={on("READ")} w={86} />
      <Conn />
      <EvBlock color={PAL.data} name="Invert L/R" icon="⇄" foot={tok ? `→ ${TOKNAME[tok]}` : "1↔3"} active={on("READ")} w={78} />
      <Conn />
      <EvBlock color={PAL.my} name="Follow_Until_Event" icon="🛣️" foot="추종 유지" active={on("FOLLOW")} w={104} />
      <Conn />
      <EvBlock color={PAL.my} name="Turn_Until_Line" icon="↻" foot="확신 회전" active={on("TURN")} w={86} />
    </LoopFrame>
  );
}

/* ── 보조 UI ── */
function Panel({ title, children }) { return <div style={{ background: COL.panel, borderRadius: 10, padding: 11, border: `1px solid ${COL.line}` }}><div style={{ fontSize: 10, letterSpacing: .5, textTransform: "uppercase", color: COL.dim, marginBottom: 8, fontWeight: 700 }}>{title}</div>{children}</div>; }
function Pill({ on, c, children }) { return <div style={{ fontSize: 11, fontWeight: 700, padding: "4px 10px", borderRadius: 20, border: `1px solid ${on ? c : COL.line}`, background: on ? `${c}22` : "transparent", color: on ? c : COL.dim }}>{children}</div>; }
function Lg({ c, children }) { return <span style={{ display: "inline-flex", alignItems: "center", gap: 4 }}><span style={{ width: 9, height: 9, borderRadius: 2, background: c }} />{children}</span>; }
function Row({ k, v, c }) { return <div style={{ display: "flex", justifyContent: "space-between", fontSize: 11.5, marginBottom: 3 }}><span style={{ color: COL.dim }}>{k}</span><b style={{ color: c || COL.ink, fontFamily: "ui-monospace,monospace" }}>{v}</b></div>; }
function PeekCase({ bits, lbl, act }) {
  return <div style={{ display: "flex", alignItems: "center", gap: 5, fontSize: 9, padding: "1px 5px", borderRadius: 4, background: act ? "rgba(31,139,255,.18)" : "#fff", border: `1px solid ${act ? "#1f8bff" : "#dde2e7"}`, boxShadow: act ? "0 0 8px rgba(31,139,255,.5)" : "none" }}>
    <b style={{ fontFamily: "ui-monospace,monospace", letterSpacing: 1, color: act ? "#1f8bff" : "#8893a2", width: 22 }}>{bits}</b>
    <span style={{ color: act ? "#222" : "#8893a2", fontWeight: 700 }}>{lbl}</span>
  </div>;
}
function ScoreBox({ label, v, done }) {
  const full = v >= 8;
  return <div style={{ flex: 1, textAlign: "center", padding: "8px 6px", borderRadius: 8, background: done ? (full ? "rgba(43,212,125,.15)" : "rgba(255,93,93,.12)") : COL.panel2, border: `1px solid ${done ? (full ? "#2bd47d" : "#ff5d5d") : COL.line}` }}>
    <div style={{ fontSize: 10, color: COL.dim }}>{label}</div>
    <div style={{ fontSize: 20, fontWeight: 800, fontFamily: "ui-monospace,monospace", color: done ? (full ? "#2bd47d" : "#ff5d5d") : COL.dim }}>{done ? `${v}/8` : "—"}</div>
  </div>;
}
function TokenStrip({ arr, n, active }) {
  return <div style={{ display: "flex", gap: 3, flexWrap: "wrap" }}>
    {arr.map((t, i) => { const shown = i < n; return <div key={i} style={{ width: 18, height: 20, borderRadius: 3, fontSize: 10, fontWeight: 800, display: "flex", alignItems: "center", justifyContent: "center", fontFamily: "ui-monospace,monospace", background: shown ? (active ? "#F6A623" : "#3a4350") : "#161b22", color: shown ? (active ? "#1a1200" : "#cfd6dd") : "#3a4350", border: `1px solid ${i === n - 1 && active ? "#fff" : "transparent"}` }}>{TOKNAME[t]}</div>; })}
  </div>;
}
function btn(bg, fg = "#0b0e12") { return { background: bg, color: fg, border: "none", padding: "8px 15px", borderRadius: 8, fontSize: 13, fontWeight: 700, cursor: "pointer", fontFamily: "system-ui,sans-serif" }; }
