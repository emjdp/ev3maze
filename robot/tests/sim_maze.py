#!/usr/bin/env python3
"""오프라인 검증 하니스 — ev3dev 없이 자율 알고리즘을 돌려본다.

solver.MazeSolver 는 주행 계층(io)에만 의존하므로, 여기서 '가짜 미로 위를 도는
로봇'(SimMotion)을 같은 io 인터페이스로 구현해 끼워 넣는다. 그러면 실제 하드웨어
없이도 알고리즘이:
  - 검증된 회전 토큰(config.EXPLORE_PLAN)과 똑같은 순서를 만드는지,
  - 노드 방문 순서가 시뮬레이터 기준(EXPECTED_EXPLORE_PATH)과 같은지,
  - RETURN 이 정확히 출발(0)로 돌아오는지
를 자동으로 확인할 수 있다.

미로 기하는 웹 시뮬레이터 src/mazeData.js 를 그대로 옮겨 온 것이다.
실행: python3 robot/tests/sim_maze.py
"""

from __future__ import print_function

import math
import os
import sys

# robot/run/ 를 import 경로에 추가 (config, solver 가 거기에 있음)
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "run"))

import config
import solver
from solver import Arrival, JUNCTION, LEAF

# --- 미로 데이터 (src/mazeData.js 와 동일) -----------------------------------
JUNCTIONS = {"A", "B", "D", "E", "F"}
EDGES = [
    ("7", "F", [[205, 138], [415, 138]]),
    ("F", "6", [[415, 138], [415, 395]]),
    ("F", "E", [[415, 138], [620, 138], [620, 395], [868, 395]]),
    ("5", "E", [[1108, 138], [868, 138], [868, 395]]),
    ("E", "D", [[868, 395], [868, 658]]),
    ("D", "4", [[868, 658], [1140, 658], [1140, 400]]),
    ("D", "3", [[868, 658], [868, 895], [648, 895]]),
    ("D", "B", [[868, 658], [388, 658]]),
    ("B", "2", [[388, 658], [148, 658], [148, 420]]),
    ("B", "A", [[388, 658], [388, 870]]),
    ("A", "1", [[388, 870], [148, 870], [148, 1110]]),
    ("A", "0", [[388, 870], [388, 1125], [1010, 1125]]),
]
EXPECTED_EXPLORE_PATH = ["0", "A", "1", "A", "B", "2", "B", "D", "4", "D", "3",
                         "D", "E", "F", "E", "5", "E", "F", "6", "F", "7"]

# 노드 색: 시작/체크포인트/도착 (실제 코스 색에 맞춘 config 값 사용)
COLORS = {"0": config.START_COLOR, "7": config.GOAL_COLOR}
for _leaf in ("1", "2", "3", "4", "5", "6"):
    COLORS[_leaf] = config.CHECKPOINT_COLOR


def _seg_angle(p0, p1):
    """선분 방향을 0/90/180/270 으로 양자화. 0=동, 90=남, 180=서, 270=북 (화면 y는 아래로 증가)."""
    deg = math.degrees(math.atan2(p1[1] - p0[1], p1[0] - p0[0]))
    return int(round(deg / 90.0)) * 90 % 360


def _build_adjacency():
    """노드별 [(이웃, 그 노드에서 떠날 때 각도, 그 이웃에 도착할 때 각도)] 목록."""
    adj = {}
    for u, v, pts in EDGES:
        for a, b, p in ((u, v, pts), (v, u, list(reversed(pts)))):
            adj.setdefault(a, []).append((
                b,
                _seg_angle(p[0], p[1]),        # 출발 각도(첫 선분)
                _seg_angle(p[-2], p[-1]),      # 도착 각도(마지막 선분)
            ))
    return adj


ADJ = _build_adjacency()


class SimMotion(object):
    """가짜 미로 위를 도는 로봇. solver 가 부르는 io 인터페이스를 구현한다."""

    def __init__(self, obstacle_node=None):
        self.cur = "0"
        # 출발에서 A 를 향한 방향으로 초기 헤딩 설정
        self.heading = next(dep for nbr, dep, _ in ADJ["0"] if nbr == "A")
        self.visits = ["0"]
        self.holding = False
        self.delivered = False
        self.obstacle_node = obstacle_node    # 이 노드에 도착하면 물체 발견(테스트용)
        self.picked_at = None

    # ---- io 인터페이스 ---------------------------------------------------
    def follow_to_node(self, label):
        nxt = None
        for nbr, dep, _arr in ADJ[self.cur]:
            if dep == self.heading:
                nxt = (nbr, _arr)
                break
        if nxt is None:
            raise RuntimeError("막다른 방향으로 주행 시도: {} heading {}".format(self.cur, self.heading))
        self.cur, self.heading = nxt[0], nxt[1]
        self.visits.append(self.cur)
        # 물체 집기(주행 중 발견 모사)
        if not self.holding and self.cur == self.obstacle_node:
            self.holding = True
            self.picked_at = self.cur
        return Arrival(JUNCTION if self.cur in JUNCTIONS else LEAF)

    def sense_exits(self):
        exits = {"L": False, "S": False, "R": False}
        for nbr, dep, _arr in ADJ[self.cur]:
            rel = (dep - self.heading) % 360
            if rel == 270:
                exits["L"] = True
            elif rel == 0:
                exits["S"] = True
            elif rel == 90:
                exits["R"] = True
        cross = exits["L"] and exits["R"] and exits["S"]
        return exits, cross

    def turn(self, token):
        delta = {solver.LEFT: -90, solver.RIGHT: 90, solver.UTURN: 180, solver.STRAIGHT: 0}[token]
        self.heading = (self.heading + delta) % 360

    def read_node_color(self):
        return COLORS.get(self.cur, 0)

    def deliver(self):
        if self.holding:
            self.holding = False
            self.delivered = True

    def finish(self, label):
        pass

    def wait_for_start(self, label):
        pass


def _check(name, ok, detail=""):
    mark = "PASS" if ok else "FAIL"
    print("[{}] {}{}".format(mark, name, "" if ok else "  -> " + detail))
    return ok


def _feed(deb, seq):
    """bits 시퀀스를 debouncer 에 차례로 먹이고, 확정된 종류 목록을 돌려준다."""
    return [deb.push(b) for b in seq]


def run_unit_tests():
    """하드웨어 없이, 센서 흔들림을 흡수하는 '순수 판정 로직'을 직접 검증한다.

    Ev3Motion 의 패턴/이벤트 판정은 solver 의 순수 함수(ArrivalDebouncer,
    majority_bit, bits_from_raw, event_kind)로 분리돼 있어 여기서 시퀀스만 먹여
    노이즈 내성(채터링·순간 유실·peek 흔들림)을 자동으로 확인할 수 있다.
    """
    from solver import (ArrivalDebouncer, PivotTracker, majority_bit, bits_from_raw,
                        event_kind, DEBOUNCE_KIND)
    ok = True

    # config 검증이 기본값에서 통과하는가
    ok &= _check("config.validate_config() 통과", config.validate_config() is True)

    # bits_from_raw: 센서별 threshold 적용 (작을수록 1=선)
    ok &= _check("bits_from_raw 센서별 threshold",
                 bits_from_raw((50, 30, 50), (40, 40, 40)) == (0, 1, 0))
    ok &= _check("bits_from_raw 비대칭 threshold",
                 bits_from_raw((35, 35, 35), (40, 30, 40)) == (1, 0, 1))

    # event_kind 기본 분류
    ok &= _check("event_kind 010=정상", event_kind((0, 1, 0)) is None)
    ok &= _check("event_kind 000=LEAF", event_kind((0, 0, 0)) == LEAF)
    ok &= _check("event_kind 110=JUNCTION", event_kind((1, 1, 0)) == JUNCTION)

    # (1) 110,111,110 흔들림이 '하나의 좌분기'로 안정 판정되는가
    deb = ArrivalDebouncer(config.JUNCTION_CONFIRM_SAMPLES, config.LEAF_CONFIRM_SAMPLES)
    noisy = _feed(deb, [(1, 1, 0), (1, 1, 1), (1, 1, 0)])
    ok &= _check("110/111/110 흔들림은 조기 확정 안 함", all(k is None for k in noisy),
                 "got {}".format(noisy))
    stable = _feed(deb, [(1, 1, 0)] * config.JUNCTION_CONFIRM_SAMPLES)
    ok &= _check("안정된 110 반복 → JUNCTION 확정", JUNCTION in stable, "got {}".format(stable))

    # (2) 010,000,010 순간 유실이 leaf 로 확정되지 않는가
    deb = ArrivalDebouncer(config.JUNCTION_CONFIRM_SAMPLES, config.LEAF_CONFIRM_SAMPLES)
    glitch = _feed(deb, [(0, 1, 0), (0, 0, 0), (0, 1, 0)])
    ok &= _check("010/000/010 순간 유실은 LEAF 아님", LEAF not in glitch, "got {}".format(glitch))

    # leaf 는 junction 보다 보수적: 같은 횟수로는 leaf 가 확정되지 않아야 한다
    deb = ArrivalDebouncer(config.JUNCTION_CONFIRM_SAMPLES, config.LEAF_CONFIRM_SAMPLES)
    short_leaf = _feed(deb, [(0, 0, 0)] * config.JUNCTION_CONFIRM_SAMPLES)
    ok &= _check("000 을 junction 횟수만큼 → 아직 LEAF 아님", LEAF not in short_leaf,
                 "got {}".format(short_leaf))
    full_leaf = _feed(deb, [(0, 0, 0)] * (config.LEAF_CONFIRM_SAMPLES -
                                          config.JUNCTION_CONFIRM_SAMPLES))
    ok &= _check("000 을 leaf 횟수만큼 → LEAF 확정", LEAF in full_leaf, "got {}".format(full_leaf))

    # (3) 회전 직후 stale 패턴 한두 번에 곧장 확정하지 않는가
    deb = ArrivalDebouncer(config.JUNCTION_CONFIRM_SAMPLES, config.LEAF_CONFIRM_SAMPLES)
    stale = deb.push((1, 0, 1))
    ok &= _check("stale 패턴 1회는 무시", stale is None)

    # (4) peek 중 111,101,111 흔들림에도 cross 판단(중앙 다수결)이 안정적인가
    ok &= _check("peek 다수결 [1,0,1] → 직진 개통", majority_bit([1, 0, 1]) == 1)
    ok &= _check("peek 다수결 [1,0,0,1,0] → 직진 막힘", majority_bit([1, 0, 0, 1, 0]) == 0)
    ok &= _check("peek 다수결 동수는 보수적(0)", majority_bit([1, 0]) == 0)

    # (4b) kind 모드: 110<->111 처럼 종류가 같은 흔들림은 누적해 더 빨리 확정
    deb = ArrivalDebouncer(config.JUNCTION_CONFIRM_SAMPLES, config.LEAF_CONFIRM_SAMPLES,
                           DEBOUNCE_KIND)
    kindseq = _feed(deb, [(1, 1, 0), (1, 1, 1), (1, 1, 0), (1, 0, 1)])
    ok &= _check("kind 모드: 분기 흔들림 누적으로 JUNCTION 확정", JUNCTION in kindseq,
                 "got {}".format(kindseq))

    # (5) _pivot 상태전이(PivotTracker) 4가지 — 하드웨어 없이 검증
    #   공통: ignore=0.1, min=0.3, require_clear=True
    def pivot_stops(seq, require_clear=True, ignore=0.1, min_s=0.3):
        t = PivotTracker(ignore, min_s, require_clear)
        return [t.update(e, c) for (e, c) in seq]

    # (a) 출발선이 계속 보이는 경우 → 선을 벗어난 적이 없으니 멈추지 않는다
    a = pivot_stops([(0.05, 1), (0.2, 1), (0.4, 1), (0.6, 1)])
    ok &= _check("pivot(a) 출발선 계속 보임 → 정지 안 함", not any(a), "got {}".format(a))
    # (b) 한 번도 선을 못 보는 경우 → 멈추지 않는다(타임아웃으로 처리됨)
    b = pivot_stops([(0.2, 0), (0.4, 0), (0.6, 0)])
    ok &= _check("pivot(b) 선을 못 봄 → 정지 안 함", not any(b), "got {}".format(b))
    # (c) 정상: 선을 벗어났다(0) min 이후 다시 잡음(1) → 그 시점에 정지
    c = pivot_stops([(0.05, 1), (0.2, 0), (0.4, 1)])
    ok &= _check("pivot(c) 정상 재포착 → 마지막에 정지", c == [False, False, True],
                 "got {}".format(c))
    # (d) U턴: 첫 선과 두 번째 선이 가깝다 → 첫 선(min 전)에선 안 멈추고 두 번째에서 멈춤
    d = pivot_stops([(0.15, 1), (0.2, 0), (0.25, 1), (0.32, 0), (0.4, 1)])
    ok &= _check("pivot(d) 첫 선 통과·두 번째 선에서 정지",
                 d == [False, False, False, False, True], "got {}".format(d))
    # require_clear=False 면 min 이후 선을 잡자마자 정지
    e = pivot_stops([(0.2, 1), (0.4, 1)], require_clear=False)
    ok &= _check("pivot require_clear=False → min 이후 즉시 정지", e == [False, True],
                 "got {}".format(e))

    # (5) validate_config 가 모순값을 잡는가 (잎이 분기보다 덜 보수적이면 오류)
    saved = config.LEAF_CONFIRM_SAMPLES
    try:
        config.LEAF_CONFIRM_SAMPLES = config.JUNCTION_CONFIRM_SAMPLES - 1
        raised = False
        try:
            config.validate_config()
        except ValueError:
            raised = True
    finally:
        config.LEAF_CONFIRM_SAMPLES = saved
    ok &= _check("validate_config 모순값(LEAF<JUNCTION) 검출", raised)
    # 복구 후 다시 통과하는지 확인
    ok &= _check("validate_config 복구 후 통과", config.validate_config() is True)

    # (6) 색 중복: 기본은 막지만 ALLOW_DUPLICATE_NODE_COLORS 로 개발 중 우회 가능
    saved_goal, saved_allow = config.GOAL_COLOR, config.ALLOW_DUPLICATE_NODE_COLORS
    try:
        config.GOAL_COLOR = config.CHECKPOINT_COLOR     # 임시로 색 충돌
        config.ALLOW_DUPLICATE_NODE_COLORS = False
        dup_blocked = False
        try:
            config.validate_config()
        except ValueError:
            dup_blocked = True
        config.ALLOW_DUPLICATE_NODE_COLORS = True        # 개발 우회
        dup_allowed = config.validate_config() is True
    finally:
        config.GOAL_COLOR, config.ALLOW_DUPLICATE_NODE_COLORS = saved_goal, saved_allow
    ok &= _check("색 중복은 기본 차단", dup_blocked)
    ok &= _check("ALLOW_DUPLICATE_NODE_COLORS=True 면 개발 중 허용", dup_allowed)

    # (7) ignore=min=0(끄기)은 timeout 안에 있으면 허용(과도하게 엄격하지 않음)
    saved = (config.LEFT_TURN_IGNORE_SECONDS, config.LEFT_TURN_MIN_SECONDS)
    try:
        config.LEFT_TURN_IGNORE_SECONDS = 0.0
        config.LEFT_TURN_MIN_SECONDS = 0.0
        zero_ok = config.validate_config() is True
    finally:
        config.LEFT_TURN_IGNORE_SECONDS, config.LEFT_TURN_MIN_SECONDS = saved
    ok &= _check("회전 ignore=min=0 허용(끄기 가능)", zero_ok)

    return ok


def run():
    ok = True

    # 0) 순수 판정 로직(센서 흔들림 흡수) 단위 검증
    print("--- 순수 판정 로직 단위 테스트 ---")
    ok &= run_unit_tests()
    print("--- 알고리즘(가짜 미로) 통합 테스트 ---")

    # 1) EXPLORE: 토큰과 노드 순서가 검증값과 일치하는가
    sim = SimMotion(obstacle_node="3")   # 노드 3 부근에서 물체를 집는 시나리오
    brain = solver.MazeSolver(sim, verbose=False)
    path = brain.explore()

    ok &= _check("EXPLORE 토큰 == config.EXPLORE_PLAN", path == list(config.EXPLORE_PLAN),
                 "got {}".format(path))
    ok &= _check("EXPLORE 노드 순서 == EXPECTED_EXPLORE_PATH",
                 sim.visits == EXPECTED_EXPLORE_PATH, "got {}".format(sim.visits))
    ok &= _check("도착(7)에서 종료", sim.cur == "7", "ended at {}".format(sim.cur))

    visited_numeric = set(n for n in sim.visits if n.isdigit())
    ok &= _check("숫자 노드 8/8 방문", visited_numeric == set("01234567"),
                 "visited {}".format(sorted(visited_numeric)))

    # 2) 물체: 도중에 집어서 도착지점에 내려놓았는가
    ok &= _check("물체 집기→도착에서 내려놓기", sim.delivered and sim.picked_at == "3",
                 "picked_at={} delivered={}".format(sim.picked_at, sim.delivered))

    # 3) RETURN: 정확히 출발(0)로 복귀하고 모든 노드를 되짚는가
    return_start_index = len(sim.visits)
    brain.return_run(path)
    return_visits = sim.visits[return_start_index:]
    ok &= _check("RETURN 출발(0)에서 종료", sim.cur == "0", "ended at {}".format(sim.cur))
    ok &= _check("RETURN 경로 == EXPLORE 역순",
                 ["7"] + return_visits == list(reversed(EXPECTED_EXPLORE_PATH)),
                 "got {}".format(return_visits))

    print("\n전체 결과:", "모두 통과 ✅" if ok else "실패 있음 ❌")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(run())
