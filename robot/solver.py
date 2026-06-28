#!/usr/bin/env python3
"""미로 자율 주행 알고리즘 (지도 없음 가정).

이 파일은 두 계층으로 나뉜다.

1. MazeSolver  — "무엇을 할지" 결정하는 순수 알고리즘.
     실제 모터/센서를 모른다. 대신 아래의 io(주행 계층) 객체에게
     "다음 노드까지 가라 / 출구를 살펴라 / 이 방향으로 돌아라" 같은
     높은 수준의 명령만 내린다. 덕분에 같은 알고리즘을 진짜 로봇과
     가짜(시뮬) 로봇 어디에도 그대로 끼워 검증할 수 있다.

2. Ev3Motion   — "어떻게 할지" 실행하는 주행 계층.
     hardware.Ev3Hardware 를 써서 반사광 라인추종, 제자리 회전,
     형태 peek, 초음파 장애물 집기 등을 실제로 수행한다.
     코스에서 튜닝하는 모든 동작 코드가 여기 모여 있다.

알고리즘 핵심(명세 미로_알고리즘_구현명세.md 2~3장):
  - 좌선우선(좌>직>우)으로 미탐색 출구 선택. 단, 형태 peek 로 십자(D형)을
    확인하면 그 분기에서는 "직진부터" 탐색한다.
  - 한 분기 우선: 고른 출구가 또 다른 분기로 이어지고 '아직 안 가본 다른 출구가
    남아 있으면' 일단 U턴해 돌아와 잎부터 끝내고, 나중에 그 분기로 내려간다.
    (남은 미탐색 출구가 없으면 곧장 내려간다 → 불필요한 왕복 없음.)
  - 분기 식별은 지도/좌표 없이 '분기 로컬 프레임'으로 한다. 어떤 분기에 처음
    도착하면 그 순간 바라보는 방향을 0°(정면), 들어온 길을 180°(부모)로 고정하고
    출구를 0/90/180/270 으로 기록한다. 잎에서 U턴해 되돌아오면 '들어온 출구가
    곧 내 뒤'라는 사실로 현재 바라보는 방향을 계산해, 남은 출구를 정확히 고른다.
    (자이로 불필요, 미로가 90° 격자라는 전제만 사용)
  - 회전 동작(1/2/3/4)을 path[] 에 차곡차곡 기록 → RETURN 때 거꾸로+좌우반전 재생.
  - 막다른 길(000)에서는 중앙센서 '컬러 모드'로 노드 색을 읽어, 도착색이면 EXPLORE
    종료, 시작색이면 RETURN 종료, 그 외는 체크포인트로 보고 U턴.
"""

from __future__ import print_function

import time
from collections import namedtuple

import config

# follow_to_node 가 돌려주는 도착 정보
Arrival = namedtuple("Arrival", ["kind"])

# ---- 회전 토큰 (path 에 저장되는 숫자) --------------------------------------
LEFT = 1
STRAIGHT = 2
RIGHT = 3
UTURN = 4
TOKEN_NAME = {LEFT: "L", STRAIGHT: "S", RIGHT: "R", UTURN: "U"}

# 도착 종류
LEAF = "LEAF"        # 막다른 길(패턴 000)
JUNCTION = "JCT"     # 분기(좌/우 중 하나 이상 갈라짐)

# ---- 분기 로컬 프레임의 각도 약속 -------------------------------------------
# 분기에 처음 도착한 순간을 기준으로:
#   0   = 정면(직진),  90 = 오른쪽,  180 = 뒤(부모, 들어온 길),  270 = 왼쪽
A_STRAIGHT, A_RIGHT, A_BACK, A_LEFT = 0, 90, 180, 270
_REL_TO_TOKEN = {A_STRAIGHT: STRAIGHT, A_RIGHT: RIGHT, A_BACK: UTURN, A_LEFT: LEFT}


# =============================================================================
# 순수(pure) 판정 계층 — 하드웨어 없이 단위 테스트 가능
# =============================================================================
# 센서 흔들림을 흡수하는 "판정 로직"만 모은다. 시간·모터가 없으므로 tests 에서
# bits 시퀀스를 그대로 먹여 동작을 검증할 수 있다(tests/sim_maze.py 참고).
# =============================================================================

def bits_from_raw(raw, thresholds):
    """raw 반사광 3개를 threshold 로 잘라 0/1 비트 3개로. (작을수록=어두울수록 1=선)"""
    return tuple(1 if v < t else 0 for v, t in zip(raw, thresholds))


def event_kind(bits):
    """이 패턴이 '결정 지점'인지: 분기(JUNCTION)/잎(LEAF)/없음(None=정상 라인 010)."""
    left, center, right = bits
    if left or right:
        return JUNCTION          # 옆으로 갈라짐 (1?? 또는 ??1)
    if not center:
        return LEAF              # 000 = 선이 끊김(막다른 길 후보)
    return None                  # 010 = 정상 라인


def majority_bit(values):
    """0/1 값들의 다수결. 동수면 0(보수적: '뚫림' 같은 적극 판정을 함부로 내지 않음)."""
    ones = sum(1 for v in values if v)
    return 1 if ones * 2 > len(values) else 0


DEBOUNCE_PATTERN = "pattern"   # 같은 bits 가 연속 동일해야 확정(노이즈에 강함)
DEBOUNCE_KIND = "kind"         # 같은 종류(JUNCTION/LEAF)면 누적(확정이 빠름)


class ArrivalDebouncer(object):
    """도착 이벤트(분기/잎)를 흔들림 속에서 확정한다(pure, 모드 선택 가능).

    mode="pattern" (기본): raw bits 자체가 연속 동일해야 확정한다. 110->111->101
        같은 회전/통과 중 흔들림은 카운트가 리셋돼 누적되지 않아 노이즈에 강하다.
    mode="kind": 같은 '종류'(둘 다 JUNCTION 이면 110<->111 이 섞여도)면 누적한다.
        분기 모양이 빠르게 떨려 패턴이 좀체 안정되지 않는 코스에서 확정이 빨라
        분기 중심을 지나치는 것을 막는다(대신 노이즈 내성은 약간 낮다).

    어느 모드든 잎(000)은 leaf_samples 로 분기보다 보수적으로 확정해, 선을 잠깐
    놓친 010->000->010 을 막다른 길로 오인하지 않는다.
    """

    def __init__(self, junction_samples, leaf_samples, mode=DEBOUNCE_PATTERN):
        self.junction_samples = junction_samples
        self.leaf_samples = leaf_samples
        self.mode = mode
        self.last_key = None
        self.count = 0

    def push(self, bits):
        """bits 한 샘플을 넣는다. 확정되면 'JUNCTION'/'LEAF', 아니면 None."""
        kind = event_kind(bits)
        # pattern 모드는 bits 자체를, kind 모드는 종류를 '동일성' 기준으로 센다.
        key = bits if self.mode == DEBOUNCE_PATTERN else kind
        if key == self.last_key:
            self.count += 1
        else:
            self.last_key = key
            self.count = 1
        if kind is None:
            return None
        needed = self.leaf_samples if kind == LEAF else self.junction_samples
        return kind if self.count >= needed else None


class PivotTracker(object):
    """제자리 회전의 '정지 시점' 판정을 순수 로직으로 분리(하드웨어 없이 테스트 가능).

    상태 전이:
      - elapsed < ignore_s : 출발 선 잔상 무시(센서 안 봄).
      - 선을 한 번이라도 벗어나면(center=0) cleared=True.
      - elapsed >= min_s AND cleared AND center=1 → 정지(라인 재포착).
    require_clear=False 면 처음부터 cleared 로 두어 '선 벗어남' 없이도 잡으면 정지.

    require_clear=True 가 핵심인 이유:
      - 좌/우 회전: 센서가 다닥다닥 붙어 출발 선 모서리를 곧장 다시 잡아도, 먼저
        선을 벗어났다 다시 잡아야 하므로 과소회전(덜 돎)을 막는다.
      - U턴: 첫 선에서 멈추지 않고 '벗어남 → 두 번째 선' 에서만 정지한다.
    """

    def __init__(self, ignore_s, min_s, require_clear):
        self.ignore_s = ignore_s
        self.min_s = min_s
        self.cleared = not require_clear

    def update(self, elapsed, center_bit):
        """elapsed(회전 시작 후 경과초)와 중앙 비트로 '지금 멈춰야 하나'를 반환."""
        if elapsed < self.ignore_s:
            return False                 # 아직 잔상 무시 구간
        if not center_bit:
            self.cleared = True          # 선을 벗어남
        return elapsed >= self.min_s and self.cleared and bool(center_bit)


def invert_token(token):
    """복귀 때 좌우를 뒤집는다. 직진/U턴은 그대로."""
    if token == LEFT:
        return RIGHT
    if token == RIGHT:
        return LEFT
    return token


def return_plan(explore_path):
    """EXPLORE 기록을 거꾸로 + 좌우반전 → RETURN 재생용 토큰 목록."""
    return [invert_token(t) for t in reversed(explore_path)]


def _token_for(angle, facing):
    """현재 바라보는 방향(facing)에서 분기-로컬 각도 angle 의 출구로 가려면 무슨 회전?"""
    return _REL_TO_TOKEN[(angle - facing) % 360]


class ExploreComplete(Exception):
    """도착(7) 색을 확인해 EXPLORE 가 끝났음을 위로 전달."""


class Aborted(Exception):
    """뒤로가기 버튼 등으로 사용자가 중지를 요청."""


class MazeSolver(object):
    """지도를 모르는 자율 주행 두뇌. io(주행 계층)에만 의존한다."""

    def __init__(self, io, verbose=True):
        self.io = io
        self.verbose = verbose
        self.path = []          # 기록된 회전 토큰들 (EXPLORE 중 채워짐)

    def _log(self, msg):
        if self.verbose:
            print(msg)

    def _record(self, token):
        self.path.append(token)

    # ======================================================================
    # EXPLORE — 출발(0)에서 모든 노드를 지나 도착(7)까지 자율 탐사
    # ======================================================================
    def explore(self):
        """성공 시 기록된 path 를 반환. 도착 색을 보면 종료."""
        self.path = []
        first = self.io.follow_to_node("EXPLORE")   # 0 을 떠나 첫 노드까지
        if first.kind == LEAF:
            # 출발 옆이 바로 막다른 길인 작은 미로(이 코스에선 없음): 색만 확인.
            if self.io.read_node_color() == config.GOAL_COLOR:
                self.io.deliver()
                return self.path
        try:
            self._explore_junction(is_root=True)
        except ExploreComplete:
            pass
        self.io.finish("EXPLORE")
        self._log("EXPLORE path = {}".format(
            " ".join(TOKEN_NAME[t] for t in self.path)))
        return self.path

    def _explore_junction(self, is_root=False):
        """현재 '분기에 막 도착해 있는' 상태에서 그 분기의 하위 전부를 탐사하고,
        끝나면 부모 쪽으로 나가 부모 분기에 되돌아온 상태로 복귀한다.

        재귀 깊이 = 미로의 분기 중첩 깊이(이 코스 최대 5) 라 매우 얕다.
        """
        exits, cross = self.io.sense_exits()   # {'L':bool,'S':bool,'R':bool}, 십자 여부
        # 분기 로컬 프레임 구성: 들어온 길(부모)=180, 나머지는 감지된 출구.
        state = {A_BACK: "parent"}
        if exits["L"]:
            state[A_LEFT] = "open"
        if exits["S"]:
            state[A_STRAIGHT] = "open"
        if exits["R"]:
            state[A_RIGHT] = "open"
        facing = 0   # 도착 순간 정면을 0 으로 둔다
        self._log("JCT 도착: 출구 {} cross={}".format(
            sorted(a for a in state if state[a] == "open"), cross))

        while True:
            angle = self._pick_open(state, facing, cross)
            if angle is not None:
                kind = self._take_exit(angle, facing)
                if kind == LEAF:
                    self._handle_leaf()                     # 색 확인(+도착이면 예외)
                    self._return_to_junction()
                    state[angle] = "leaf"
                else:  # 가 보니 분기였다
                    if self._has_open_other(state, angle):
                        # 방금 간 출구 말고도 안 가본 출구(잎일 수 있음)가 남았으면
                        # → 한 분기 우선: 일단 보류(U턴 복귀)하고 잎부터 끝낸다
                        self._record(UTURN)
                        self.io.turn(UTURN)
                        self._return_to_junction()
                        state[angle] = "deferred"
                    else:
                        # 남은 미탐색 출구가 없으면 → 불필요한 왕복 없이 곧장 내려감
                        self._explore_junction(is_root=False)
                        state[angle] = "done"
                facing = (angle + A_BACK) % 360   # 그 출구에서 되돌아온 방향
                continue

            angle = self._pick_deferred(state, facing)
            if angle is not None:
                self._descend(angle, facing)
                state[angle] = "done"
                facing = (angle + A_BACK) % 360
                continue

            # 모든 출구 완료 → 부모로 나간다(루트면 더 나갈 부모가 없으니 종료)
            if is_root:
                return
            token = _token_for(A_BACK, facing)
            self._record(token)
            self.io.turn(token)
            self.io.follow_to_node("EXPLORE")   # 부모 분기에 도착
            return

    # ---- 선택 규칙 -------------------------------------------------------
    def _pick_open(self, state, facing, cross):
        """미탐색('open') 출구를 우선순위로 고른다.
        십자(D형)면 '직진 우선', 그 외는 '좌선우선'."""
        order = (A_STRAIGHT, A_LEFT, A_RIGHT) if cross else (A_LEFT, A_STRAIGHT, A_RIGHT)
        opens = [a for a in state if state[a] == "open"]
        for rel in order:
            for a in opens:
                if (a - facing) % 360 == rel:
                    return a
        return None

    def _pick_deferred(self, state, facing):
        """보류해 둔 분기 출구를 좌선우선으로 고른다."""
        order = (A_LEFT, A_STRAIGHT, A_RIGHT)
        defers = [a for a in state if state[a] == "deferred"]
        for rel in order:
            for a in defers:
                if (a - facing) % 360 == rel:
                    return a
        return None

    def _has_open_other(self, state, angle):
        """방금 선택한 angle 을 제외하고, 아직 미탐색('open')인 출구가 남았는지."""
        return any(a != angle and v == "open" for a, v in state.items())

    # ---- 동작 묶음 -------------------------------------------------------
    def _take_exit(self, angle, facing):
        """출구로 회전 후 다음 노드까지 주행. 도착 종류(LEAF/JUNCTION) 반환."""
        token = _token_for(angle, facing)
        self._record(token)
        self.io.turn(token)
        arr = self.io.follow_to_node("EXPLORE")
        return arr.kind

    def _handle_leaf(self):
        """막다른 길: 색을 읽어 도착이면 종료, 아니면 체크포인트로 보고 U턴 준비."""
        color = self.io.read_node_color()
        if color == config.GOAL_COLOR:
            self._log("도착 색 확인 → EXPLORE 종료")
            self.io.deliver()              # 들고 있던 물체를 도착지점에 내려놓음
            raise ExploreComplete()
        self._log("체크포인트(색 {}) → U턴".format(color))
        self._record(UTURN)
        self.io.turn(UTURN)

    def _return_to_junction(self):
        """U턴 직후, 라인을 따라 분기로 되돌아온다."""
        self.io.follow_to_node("EXPLORE")

    def _descend(self, angle, facing):
        """보류 분기로 회전·주행해 그 분기로 내려가 재귀 탐사."""
        token = _token_for(angle, facing)
        self._record(token)
        self.io.turn(token)
        self.io.follow_to_node("EXPLORE")   # 자식 분기에 도착
        self._explore_junction(is_root=False)  # 자식이 끝나면 여기로 되돌아온 상태로 복귀

    # ======================================================================
    # RETURN — 도착(7)에서 기록을 거꾸로+반전 재생해 출발(0)로 복귀
    # ======================================================================
    def return_run(self, explore_path=None):
        plan = return_plan(explore_path if explore_path is not None else self.path)
        self._log("RETURN plan = {}".format(" ".join(TOKEN_NAME[t] for t in plan)))
        # 도착(7)은 막다른 길이라 들어온 쪽을 향하고 있다 → 먼저 돌아서 출발 방향으로.
        self.io.turn(UTURN)
        arr = self.io.follow_to_node("RETURN")   # 첫 노드까지
        for token in plan:
            # 막다른 길에서만 색을 확인한다(분기에서 컬러 모드 전환은 불필요·느림).
            if arr.kind == LEAF and self.io.read_node_color() == config.START_COLOR:
                break                            # 출발 색을 보면 끝
            self.io.turn(token)
            arr = self.io.follow_to_node("RETURN")
        self.io.finish("RETURN")
        return plan

    # ======================================================================
    # 폴백: 검증된 고정 플랜 재생 (--plan). 센서 판단/peek 없이 회전만 기록대로.
    # ======================================================================
    def run_plan(self, plan, final_label="EXPLORE"):
        for token in plan:
            self.io.follow_to_node(final_label)
            self.io.turn(token)
        self.io.follow_to_node(final_label)
        self.io.finish(final_label)
        return list(plan)


# =============================================================================
# Ev3Motion — 실제 주행 계층 (위 MazeSolver 가 부르는 io 구현체)
# =============================================================================
# 코스에서 튜닝하는 "어떻게 움직이나" 코드가 전부 여기 모인다. 알고리즘(MazeSolver)
# 은 이 클래스의 메서드만 호출하므로, 같은 인터페이스의 가짜 구현(tests/sim_maze.py)
# 으로 바꿔 끼우면 ev3dev 없이도 알고리즘을 검증할 수 있다.
# =============================================================================
class Ev3Motion(object):
    def __init__(self, hardware, verbose=True):
        self.hw = hardware
        self.verbose = verbose
        self._last_debug_t = 0.0       # DEBUG_SENSOR_LOG 주기 제한용
        self._last_dist_debug_t = 0.0  # DEBUG_DISTANCE 주기 제한용(센서로그와 분리)

    def _log(self, msg):
        if self.verbose:
            print(msg)

    # ---- 저수준 읽기/구동 (모든 패턴 판정은 순수 함수에 위임) -------------
    def _read(self):
        """(raw 3개, 0/1 비트 3개) 반환. raw 는 비례제어용, 비트는 패턴 판정용."""
        raw = self.hw.read_reflect()
        bits = bits_from_raw(raw, config.line_thresholds())
        self._debug_sensor(raw, bits)
        return raw, bits

    def _drive(self, left_speed, right_speed, trim=True):
        """구동. trim=True 면 좌우 모터 편차 보정 적용(직진/추종용).
        제자리 회전은 trim=False 로 줘서 트림이 회전 거동을 바꾸지 않게 한다."""
        self.hw.drive(left_speed, right_speed, apply_trim=trim)

    def _stop(self):
        """정지 + 관성/브레이크 settle. 정지 직후 읽는 패턴이 잔상이 되지 않게."""
        self.hw.stop()
        if config.POST_STOP_SETTLE_SECONDS:
            time.sleep(config.POST_STOP_SETTLE_SECONDS)

    def _check_abort(self):
        if self.hw.abort_requested():
            raise Aborted()

    # ---- 디버그 로그 (주기 제한) -----------------------------------------
    def _debug_sensor(self, raw, bits):
        if not config.DEBUG_SENSOR_LOG:
            return
        now = time.time()
        if now - self._last_debug_t < config.DEBUG_LOG_INTERVAL_SECONDS:
            return
        self._last_debug_t = now
        print("    [SENSOR] raw={} bits={}".format(tuple(raw), bits))

    # ---- io 인터페이스: 다음 노드까지 주행 -------------------------------
    def follow_to_node(self, label):
        """중앙센서 반사광으로 라인을 따라가다 분기/잎을 만나면 멈추고 Arrival 반환.
        주행 중(EXPLORE) 초음파로 물체를 발견하면 즉시 집는다(한 번만).

        - 도착 이벤트는 ArrivalDebouncer 로 확정한다(EVENT_DEBOUNCE_MODE 로 모드 선택).
        - 000(잎 후보)은 곧장 막다른 길로 보지 않고, '짧게만' 복구 전진해 재포착을
          시도한 뒤 멈춰서 제자리 재샘플한다(막다른 벽을 들이받지 않도록 보수적).
          LEAF_CONFIRM_SAMPLES + LOST_LINE_RECOVERY_SECONDS 를 모두 만족할 때만 확정.
        - 분기 확정 시 JUNCTION_CENTERING_SECONDS 만큼 더 전진해 분기 중심에 정렬.
        """
        self._clear_junction()      # 직전 회전으로 머문 분기를 살짝 벗어남
        deb = ArrivalDebouncer(config.JUNCTION_CONFIRM_SAMPLES, config.LEAF_CONFIRM_SAMPLES,
                               config.EVENT_DEBOUNCE_MODE)
        leaf_since = None
        while True:
            self._check_abort()
            self._maybe_pick_object(label)
            raw, bits = self._read()
            ev = event_kind(bits)
            confirmed = deb.push(bits)

            if ev == LEAF:
                # 선을 놓친 것일 수 있다 → 아주 잠깐만 저속 전진해 재포착을 노리고,
                # 복구 시간이 지나면 더 전진하지 말고 멈춰서 제자리 재샘플한다
                # (계속 전진하면 진짜 막다른 길에서 벽을 들이받는다).
                if leaf_since is None:
                    leaf_since = time.time()
                recovered = (time.time() - leaf_since) >= config.LOST_LINE_RECOVERY_SECONDS
                if confirmed == LEAF and recovered:
                    self._arrive(label, LEAF, bits, center=False)
                    return Arrival(LEAF)
                if recovered:
                    self.hw.stop()      # 복구 끝: 더 전진 말고 제자리에서 재샘플
                else:
                    self._drive(config.LOST_LINE_RECOVERY_SPEED, config.LOST_LINE_RECOVERY_SPEED)
            else:
                leaf_since = None
                if confirmed == JUNCTION:
                    self._arrive(label, JUNCTION, bits, center=True)
                    return Arrival(JUNCTION)
                if ev == JUNCTION:
                    # 확정 전까진 천천히 전진해 분기 중심에 자리잡는다.
                    self._drive(config.FOLLOW_SPEED, config.FOLLOW_SPEED)
                else:
                    lspeed, rspeed = self._follow_speeds(raw, bits)
                    self._drive(lspeed, rspeed)
            time.sleep(config.LOOP_DELAY)

    def _arrive(self, label, kind, bits, center):
        """도착 확정 처리: (분기면) 중심 정렬 → 정지/settle → 로그."""
        if center and config.JUNCTION_CENTERING_SECONDS > 0:
            self._drive(config.FOLLOW_SPEED, config.FOLLOW_SPEED)
            time.sleep(config.JUNCTION_CENTERING_SECONDS)
        self._stop()
        if config.DEBUG_EVENTS:
            print("    [EVENT] {} 확정 bits={}".format(kind, bits))
        self._log("  -> {} 도착 ({}) bits={}".format(label, kind, bits))

    def _follow_speeds(self, raw, bits):
        """비례 제어: 중앙선에서 벗어난 정도에 비례해 좌우 속도를 조정."""
        correction = (raw[2] - raw[0]) * config.KP
        if bits[0] and not bits[2]:
            correction = abs(correction) + config.SIDE_CORRECTION
        elif bits[2] and not bits[0]:
            correction = -(abs(correction) + config.SIDE_CORRECTION)
        return config.FOLLOW_SPEED - correction, config.FOLLOW_SPEED + correction

    def _clear_junction(self):
        """회전 직후 분기 위에 머물러 생기는 거짓 이벤트를 막으려 짧게 직진."""
        self._drive(config.FOLLOW_SPEED, config.FOLLOW_SPEED)
        time.sleep(config.CLEAR_JUNCTION_SECONDS)
        if config.POST_MOVE_SENSOR_SETTLE_SECONDS:
            time.sleep(config.POST_MOVE_SENSOR_SETTLE_SECONDS)

    # ---- io 인터페이스: 분기 출구 감지(형태 peek 포함) -------------------
    def sense_exits(self):
        """현재 분기에서 좌/직/우 출구가 열렸는지 판단. 좌·우가 동시에 보이면
        살짝 직진해(peek) 중앙이 뚫렸는지 확인한다. (분기당 1회만 호출됨)
        반환: ({'L','S','R': bool}, cross) — cross=십자(D형) 여부."""
        if config.PEEK_SENSOR_SETTLE_SECONDS:
            time.sleep(config.PEEK_SENSOR_SETTLE_SECONDS)
        _, bits = self._read()
        left, right = bool(bits[0]), bool(bits[2])
        if left and right:
            # 1?1: 형태가 헷갈리므로 살짝 전진해 중앙센서로 직진 개통 확인.
            self._drive(config.FOLLOW_SPEED, config.FOLLOW_SPEED)
            time.sleep(config.PEEK_FORWARD_SECONDS)
            self._stop()
            if config.PEEK_SETTLE_SECONDS:
                time.sleep(config.PEEK_SETTLE_SECONDS)
            # 한 번이 아니라 여러 번 읽어 다수결로 직진 개통을 판정(순간 노이즈 방지).
            center_samples = []
            for _ in range(config.PEEK_SAMPLES):
                _, pk = self._read()
                center_samples.append(pk[1])
                time.sleep(config.LOOP_DELAY)
            straight = bool(majority_bit(center_samples))
            if config.DEBUG_EVENTS:
                print("    [PEEK] center_samples={} -> straight={}".format(center_samples, straight))
            # 제자리로 후진 복귀(전진량과 따로 튜닝).
            self._drive(-config.FOLLOW_SPEED, -config.FOLLOW_SPEED)
            time.sleep(config.PEEK_BACK_SECONDS)
            self._stop()
            if config.PEEK_SETTLE_SECONDS:
                time.sleep(config.PEEK_SETTLE_SECONDS)
            cross = straight                 # 좌·우 + 직진 = 십자(D형)
        else:
            straight = bool(bits[1])
            cross = False
        return {"L": left, "S": straight, "R": right}, cross

    # ---- io 인터페이스: 회전(라인 재포착 방식) ---------------------------
    def turn(self, token):
        """좌/우/U턴을 각각의 속도·ignore·min·timeout 으로 돈다(실기 보정값 분리).

        직진(S)은 회전이 아니라 분기를 지나 다음 라인에 올라타기 위한 nudge 다.
        """
        if config.DEBUG_TURNS:
            print("    [TURN] start {}".format(TOKEN_NAME[token]))
        self._log("  turn {}".format(TOKEN_NAME[token]))
        if token == STRAIGHT:
            self._drive(config.FOLLOW_SPEED, config.FOLLOW_SPEED)
            time.sleep(config.STRAIGHT_NUDGE_SECONDS)
            self._stop()
            if config.POST_MOVE_SENSOR_SETTLE_SECONDS:
                time.sleep(config.POST_MOVE_SENSOR_SETTLE_SECONDS)
        elif token == LEFT:
            self._pivot(-config.LEFT_TURN_SPEED, config.LEFT_TURN_SPEED,
                        config.LEFT_TURN_IGNORE_SECONDS, config.LEFT_TURN_MIN_SECONDS,
                        config.LEFT_TURN_TIMEOUT_SECONDS, config.LEFT_TURN_REQUIRE_LINE_CLEAR)
        elif token == RIGHT:
            self._pivot(config.RIGHT_TURN_SPEED, -config.RIGHT_TURN_SPEED,
                        config.RIGHT_TURN_IGNORE_SECONDS, config.RIGHT_TURN_MIN_SECONDS,
                        config.RIGHT_TURN_TIMEOUT_SECONDS, config.RIGHT_TURN_REQUIRE_LINE_CLEAR)
        elif token == UTURN:
            self._pivot(config.UTURN_SPEED, -config.UTURN_SPEED,
                        config.UTURN_IGNORE_SECONDS, config.UTURN_MIN_SECONDS,
                        config.UTURN_TIMEOUT_SECONDS, config.UTURN_REQUIRE_LINE_CLEAR)
        else:
            raise ValueError("unknown token {}".format(token))
        if config.DEBUG_TURNS:
            print("    [TURN] done {}".format(TOKEN_NAME[token]))

    def _pivot(self, left_speed, right_speed, ignore_s, min_s, timeout_s, require_clear):
        """제자리 회전 → 라인 재포착으로 각도를 맞춘다(자이로 불필요).

        (1) IGNORE 동안은 센서를 보지 않고 회전만(출발 선 잔상 무시).
        (2) MIN 전에는 선을 잡아도 정지하지 않음(과소회전 방지 = 최소 회전 시간).
        (3) require_clear 면 '선을 한 번 벗어났다가' 다시 잡을 때만 정지로 인정
            (센서가 다닥다닥 붙어 출발 선 모서리를 곧장 다시 잡는 것 방지; U턴 필수).
        (4) 정지 후 POST_TURN settle 로 브레이크·센서 안정화.
        """
        start = time.time()
        self._drive(left_speed, right_speed, trim=False)   # 회전엔 직진용 트림 미적용
        deadline = start + timeout_s
        tracker = PivotTracker(ignore_s, min_s, require_clear)
        while time.time() < deadline:
            self._check_abort()
            elapsed = time.time() - start
            if elapsed < ignore_s:
                time.sleep(config.LOOP_DELAY)
                continue
            _, bits = self._read()
            if tracker.update(elapsed, bits[1]):
                self._stop()
                self._post_turn_settle()
                return
            time.sleep(config.LOOP_DELAY)
        self._stop()
        raise RuntimeError("회전 후 라인을 다시 잡지 못했습니다(타임아웃)")

    def _post_turn_settle(self):
        """회전 정지 후 기계적 settle + 센서 안정화(회전 잔상 패턴 방지)."""
        if config.POST_TURN_SETTLE_SECONDS:
            time.sleep(config.POST_TURN_SETTLE_SECONDS)
        if config.POST_TURN_SENSOR_SETTLE_SECONDS:
            time.sleep(config.POST_TURN_SENSOR_SETTLE_SECONDS)

    # ---- io 인터페이스: 노드 색 판정 -------------------------------------
    def read_node_color(self):
        """막다른 길에서 멈춰 중앙센서 컬러 모드로 노드 색(0~7)을 안정적으로 읽는다.

        컬러 모드 전환 직후 값이 튀므로 (hardware 가) settle + 더미 읽기로 안정화한
        뒤, 같은 색이 연속 COLOR_CONFIRM_SAMPLES 번 보일 때만 확정한다.
        """
        last, count = None, 0
        while True:
            self._check_abort()
            color = self.hw.read_center_color()
            if config.DEBUG_COLOR:
                print("    [COLOR] read={} (start={} checkpoint={} goal={})".format(
                    color, config.START_COLOR, config.CHECKPOINT_COLOR, config.GOAL_COLOR))
            if color == last:
                count += 1
                if count >= config.COLOR_CONFIRM_SAMPLES:
                    self._log("  색 판정: {}".format(color))
                    return color
            else:
                last, count = color, 1
            time.sleep(config.LOOP_DELAY)

    # ---- io 인터페이스: 물체 집기/내려놓기 -------------------------------
    def _maybe_pick_object(self, label="EXPLORE"):
        if self.hw.holding_object:
            return
        # RETURN 때는 이미 내려놓았으므로 벽/구조물 오탐을 막기 위해 감지하지 않는다.
        if config.OBJECT_DETECT_ON_EXPLORE_ONLY and label != "EXPLORE":
            return
        if not self._object_in_range():
            return
        # 한 번 튄 값에 속지 않도록 연속 확인
        for _ in range(config.OBSTACLE_CONFIRM_SAMPLES - 1):
            time.sleep(config.LOOP_DELAY)
            if not self._object_in_range():
                return
        self._stop()
        self._log("  물체 발견 → 집기")
        self.hw.grip()
        if config.POST_GRIP_SETTLE_SECONDS:
            time.sleep(config.POST_GRIP_SETTLE_SECONDS)   # 집은 뒤 자세 흔들림 안정화

    def _object_in_range(self):
        """전방 물체가 집기 거리 안인가. 비현실적으로 작은 값(벽/부품 반사)은 노이즈로 무시."""
        d = self.hw.distance_cm()
        if config.DEBUG_DISTANCE:
            now = time.time()
            if now - self._last_dist_debug_t >= config.DEBUG_LOG_INTERVAL_SECONDS:
                self._last_dist_debug_t = now
                print("    [DIST] {} cm".format(d))
        return config.OBSTACLE_MIN_VALID_CM <= d < config.OBSTACLE_DISTANCE_CM

    def deliver(self):
        if self.hw.holding_object:
            self._log("  도착지점에 물체 내려놓기")
            self.hw.release()
            if config.POST_RELEASE_SETTLE_SECONDS:
                time.sleep(config.POST_RELEASE_SETTLE_SECONDS)

    # ---- io 인터페이스: 단계 시작/종료 -----------------------------------
    def wait_for_start(self, label):
        if not config.WAIT_FOR_BUTTON:
            return
        print("가운데 버튼={} 시작, 뒤로가기=중지".format(label))
        while True:
            if self.hw.abort_requested():
                raise Aborted()
            if self.hw.enter_pressed():
                time.sleep(0.35)
                return
            time.sleep(0.05)

    def finish(self, label):
        self.hw.stop()
        self.hw.beep()
        print("{} 완료".format(label))
