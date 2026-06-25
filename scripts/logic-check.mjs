import assert from "node:assert/strict";
import { EXPECTED_EXPLORE_PATH } from "../src/mazeData.js";
import { buildLeg, createSimulation, explorePath, invertToken, TOK } from "../src/mazeLogic.js";

const explore = explorePath();
assert.deepEqual(explore, EXPECTED_EXPLORE_PATH, "EXPLORE path must match the implementation spec trace");

const sim = createSimulation();
const exploreLeg = buildLeg(explore);
const moveLog = exploreLeg.decisions.filter((decision) => decision.recorded).map((decision) => TOK[decision.action]);
assert.deepEqual(sim.moveLog, moveLog, "MOVELOG must be derived from decision actions");
assert.deepEqual(
  sim.revInv,
  sim.moveLog.slice().reverse().map(invertToken),
  "RETURN tokens must be reversed and left/right inverted",
);

const seen = {};
const peekEvents = [];
const skippedPeek = [];
for (const decision of exploreLeg.decisions) {
  if (decision.kind !== "JCT") continue;
  const seesLeftAndRight = decision.pattern[0] === 1 && decision.pattern[2] === 1;
  if (seesLeftAndRight && !seen[decision.node]) peekEvents.push(decision.node);
  if (seesLeftAndRight && seen[decision.node]) skippedPeek.push(decision.node);
  seen[decision.node] = true;
}
assert.deepEqual(peekEvents, ["B", "D"], "Only B and D should perform first-visit peek");
assert.ok(skippedPeek.includes("D"), "D revisits must skip peek after seen[D]=1");

assert.ok(
  explore.join(",").includes("E,F,E,5,E,F"),
  "E must discover F, return to E, process 5, then continue through F",
);

console.log("logic-check: ok");
