export const NODES = {
  "0": { x: 1010, y: 1125, kind: "start", color: "#f4c20d", label: "0" },
  "1": { x: 148, y: 1110, kind: "leaf", color: "#1c5fe0", label: "1" },
  "2": { x: 148, y: 420, kind: "leaf", color: "#1c5fe0", label: "2" },
  "3": { x: 648, y: 895, kind: "leaf", color: "#1c5fe0", label: "3" },
  "4": { x: 1140, y: 400, kind: "leaf", color: "#1c5fe0", label: "4" },
  "5": { x: 1108, y: 138, kind: "leaf", color: "#1c5fe0", label: "5" },
  "6": { x: 415, y: 395, kind: "leaf", color: "#1c5fe0", label: "6" },
  "7": { x: 205, y: 138, kind: "goal", color: "#1c5fe0", label: "7" },
  A: { x: 388, y: 870, kind: "junction" },
  B: { x: 388, y: 658, kind: "junction" },
  D: { x: 868, y: 658, kind: "junction" },
  E: { x: 868, y: 395, kind: "junction" },
  F: { x: 415, y: 138, kind: "junction" },
};

export const EDGES = [
  ["7", "F", [[205, 138], [415, 138]]],
  ["F", "6", [[415, 138], [415, 395]]],
  ["F", "E", [[415, 138], [620, 138], [620, 395], [868, 395]]],
  ["5", "E", [[1108, 138], [868, 138], [868, 395]]],
  ["E", "D", [[868, 395], [868, 658]]],
  ["D", "4", [[868, 658], [1140, 658], [1140, 400]]],
  ["D", "3", [[868, 658], [868, 895], [648, 895]]],
  ["D", "B", [[868, 658], [388, 658]]],
  ["B", "2", [[388, 658], [148, 658], [148, 420]]],
  ["B", "A", [[388, 658], [388, 870]]],
  ["A", "1", [[388, 870], [148, 870], [148, 1110]]],
  ["A", "0", [[388, 870], [388, 1125], [1010, 1125]]],
];

export const SENSOR_CFG = {
  B: {
    L: { f: 44, l: 38 },
    C: { f: 44, l: 0 },
    R: { f: 44, l: -38 },
  },
  C: {
    L: { f: 30, l: 38 },
    C: { f: 62, l: 0 },
    R: { f: 30, l: -38 },
  },
};

export const NUMERIC_NODES = ["0", "1", "2", "3", "4", "5", "6", "7"];
export const EXPECTED_EXPLORE_PATH = ["0", "A", "1", "A", "B", "2", "B", "D", "4", "D", "3", "D", "E", "F", "E", "5", "E", "F", "6", "F", "7"];

const SCALE = 0.42;
const MARGIN = 40;
let minX = Infinity;
let minY = Infinity;
let maxX = -Infinity;
let maxY = -Infinity;

EDGES.forEach(([, , pts]) => {
  pts.forEach(([x, y]) => {
    minX = Math.min(minX, x);
    minY = Math.min(minY, y);
    maxX = Math.max(maxX, x);
    maxY = Math.max(maxY, y);
  });
});

export const CANVAS = {
  width: (maxX - minX) * SCALE + MARGIN * 2,
  height: (maxY - minY) * SCALE + MARGIN * 2,
  tx: (x) => (x - minX) * SCALE + MARGIN,
  ty: (y) => (y - minY) * SCALE + MARGIN,
};
