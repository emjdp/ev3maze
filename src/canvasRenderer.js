import { CANVAS, EDGES, NODES, SENSOR_CFG } from "./mazeData.js";
import { COLORS, PALETTE } from "./theme.js";

export function classify(r, g, b) {
  const refl = Math.round(((0.299 * r + 0.587 * g + 0.114 * b) / 255) * 100);
  let name = "WHITE";
  if (r > 200 && g > 160 && b < 120) name = "YEL";
  else if (b > 150 && r < 140) name = "BLU";
  else if (refl < 35) name = "BLK";
  return { refl, name, bit: refl < 40 ? 1 : 0 };
}

export function renderMazeBitmap() {
  const off = document.createElement("canvas");
  off.width = Math.ceil(CANVAS.width);
  off.height = Math.ceil(CANVAS.height);
  const ctx = off.getContext("2d");

  ctx.fillStyle = COLORS.surface;
  ctx.fillRect(0, 0, off.width, off.height);
  ctx.strokeStyle = "#141414";
  ctx.lineWidth = 13;
  ctx.lineCap = "round";
  ctx.lineJoin = "round";

  EDGES.forEach(([, , pts]) => {
    ctx.beginPath();
    pts.forEach(([x, y], i) => {
      if (i) ctx.lineTo(CANVAS.tx(x), CANVAS.ty(y));
      else ctx.moveTo(CANVAS.tx(x), CANVAS.ty(y));
    });
    ctx.stroke();
  });

  Object.entries(NODES).forEach(([, node]) => {
    if (node.kind === "junction") return;
    const r = node.kind === "start" || node.kind === "goal" ? 15 : 13;
    ctx.beginPath();
    ctx.arc(CANVAS.tx(node.x), CANVAS.ty(node.y), r, 0, Math.PI * 2);
    ctx.fillStyle = node.color;
    ctx.fill();
    if (node.kind === "goal") {
      ctx.lineWidth = 3.5;
      ctx.strokeStyle = "#2bd47d";
      ctx.stroke();
    }
  });

  return {
    off,
    data: ctx.getImageData(0, 0, off.width, off.height).data,
    w: off.width,
    h: off.height,
  };
}

export function sampleMaze(mx, my, bitmap) {
  if (!bitmap) return classify(244, 243, 238);
  const px = Math.round(CANVAS.tx(mx));
  const py = Math.round(CANVAS.ty(my));
  if (px < 0 || py < 0 || px >= bitmap.w || py >= bitmap.h) return classify(244, 243, 238);
  const i = (py * bitmap.w + px) * 4;
  return classify(bitmap.data[i], bitmap.data[i + 1], bitmap.data[i + 2]);
}

export function drawSimulationCanvas(canvas, bitmap, state, sensorCfgKey) {
  if (!canvas || !bitmap || !state) return null;

  const ctx = canvas.getContext("2d");
  ctx.clearRect(0, 0, CANVAS.width, CANVAS.height);
  ctx.drawImage(bitmap.off, 0, 0);

  Object.keys(state.visited).forEach((id) => {
    const node = NODES[id];
    if (!node || node.kind === "junction") return;
    ctx.beginPath();
    ctx.arc(CANVAS.tx(node.x), CANVAS.ty(node.y), 19, 0, Math.PI * 2);
    ctx.strokeStyle = "rgba(43,212,125,.62)";
    ctx.lineWidth = 2.4;
    ctx.stroke();
  });

  ctx.font = "700 11px ui-monospace, SFMono-Regular, Menlo, monospace";
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  Object.entries(NODES).forEach(([id, node]) => {
    if (node.kind === "junction") {
      ctx.fillStyle = COLORS.dim;
      ctx.fillText(id, CANVAS.tx(node.x) + 14, CANVAS.ty(node.y) - 14);
    } else {
      ctx.fillStyle = node.kind === "start" ? "#332600" : "#ffffff";
      ctx.fillText(node.label, CANVAS.tx(node.x), CANVAS.ty(node.y));
    }
  });

  const cfg = SENSOR_CFG[sensorCfgKey];
  const ch = Math.cos(state.heading);
  const sh = Math.sin(state.heading);
  const sensorWorld = (offset) => [state.x + offset.f * ch + offset.l * -sh, state.y + offset.f * sh + offset.l * ch];
  const sL = sensorWorld(cfg.L);
  const sC = sensorWorld(cfg.C);
  const sR = sensorWorld(cfg.R);
  const rL = sampleMaze(...sL, bitmap);
  const rC = sampleMaze(...sC, bitmap);
  const rR = sampleMaze(...sR, bitmap);

  const rx = CANVAS.tx(state.x);
  const ry = CANVAS.ty(state.y);
  ctx.save();
  ctx.translate(rx, ry);
  ctx.rotate(state.heading);
  if (state.phase === "obstacle") {
    ctx.fillStyle = "#ff8a3d";
    ctx.fillRect(20, -10, 14, 20);
  }
  ctx.fillStyle = "#303946";
  ctx.strokeStyle = PALETTE.flow;
  ctx.lineWidth = 2;
  roundedRect(ctx, -14, -11, 28, 22, 5);
  ctx.fill();
  ctx.stroke();
  ctx.fillStyle = "#090b0f";
  ctx.fillRect(-8, -14, 10, 4);
  ctx.fillRect(-8, 10, 10, 4);
  ctx.restore();

  drawSensor(ctx, sL, rL);
  drawSensor(ctx, sR, rR);
  drawSensor(ctx, sC, rC);

  return {
    bits: [rL.bit, rC.bit, rR.bit],
    refl: { L: rL.refl, C: rC.refl, R: rR.refl },
  };
}

function drawSensor(ctx, point, reading) {
  const color = reading.name === "BLU" ? "#3b82f6" : reading.name === "YEL" ? "#f4c20d" : reading.name === "BLK" ? "#111111" : "#cfd6dd";
  ctx.beginPath();
  ctx.arc(CANVAS.tx(point[0]), CANVAS.ty(point[1]), 4.2, 0, Math.PI * 2);
  ctx.fillStyle = color;
  ctx.fill();
  ctx.lineWidth = 1.4;
  ctx.strokeStyle = reading.name === "WHITE" ? "#94a0ad" : "#ffffff";
  ctx.stroke();
}

function roundedRect(ctx, x, y, width, height, radius) {
  if (typeof ctx.roundRect === "function") {
    ctx.beginPath();
    ctx.roundRect(x, y, width, height, radius);
    return;
  }

  ctx.beginPath();
  ctx.moveTo(x + radius, y);
  ctx.arcTo(x + width, y, x + width, y + height, radius);
  ctx.arcTo(x + width, y + height, x, y + height, radius);
  ctx.arcTo(x, y + height, x, y, radius);
  ctx.arcTo(x, y, x + width, y, radius);
  ctx.closePath();
}
