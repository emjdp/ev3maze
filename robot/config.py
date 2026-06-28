"""EV3 미로 로봇 튜닝 값 (ev3dev-stretch).

모든 "바꿀 만한 값"은 이 파일 한 곳에 모았다. 코드(solver.py/hardware.py)는
숫자를 직접 박지 않고 전부 여기서 가져다 쓴다. 실제 코스에서 조정할 때는
이 파일만 수정하면 된다. 각 값이 "무엇을 의미하는지", 그리고 "어떤 증상일 때
올리고/내리는지"를 주석으로 적어 두었다.

실기 튜닝 순서(자세한 내용은 ALGORITHM.md 9장):
  1) 센서 흑/백 raw 값 측정 → 2) 라인 threshold → 3) 직진 속도/KP →
  4) 좌/우 90도 회전 → 5) U턴 → 6) 분기/peek 안정화 → 7) 컬러 판정 → 8) 초음파/그립

파일 맨 아래의 validate_config() 는 말이 안 되는 값(음수, timeout<=ignore 등)을
조기에 잡아 준다. main.py 가 시작할 때 호출한다.
"""

# =============================================================================
# 1. 하드웨어 포트 (실제 배선과 반드시 일치)
# =============================================================================
# 주행용 라지 모터: 왼쪽 A, 오른쪽 B
DRIVE_LEFT_PORT = "outA"
DRIVE_RIGHT_PORT = "outB"
# 그립&리프트용 미디엄 모터: C
GRIP_MOTOR_PORT = "outC"

# 컬러센서 3개: 왼쪽부터 1, 2, 3 (중앙 = 라인추종 + 노드색 판정 겸용)
COLOR_LEFT_PORT = "in1"
COLOR_CENTER_PORT = "in2"
COLOR_RIGHT_PORT = "in3"
# 초음파(장애물/물체) 센서: 4
ULTRASONIC_PORT = "in4"

# =============================================================================
# 2. 라인추종(주행) — 중앙센서 반사광 기반
# =============================================================================
# 반사광(0~100). 이 값보다 어두우면(작으면) "검은 선"으로 본다.
# 흰 바닥과 검은 선의 중간값으로 캘리브레이션할 것.
LINE_THRESHOLD = 40
# 센서 3개의 반사광 특성(높이·각도·LED 편차·주변광)이 서로 다를 수 있다.
# 한 개 LINE_THRESHOLD 로 부족하면 좌/중/우를 따로 잡는다. None 이면 LINE_THRESHOLD 사용.
#   증상: 특정 센서만 흰 바닥을 검정으로(또는 그 반대로) 오판 → 그 센서 값만 조정.
LEFT_LINE_THRESHOLD = None
CENTER_LINE_THRESHOLD = None
RIGHT_LINE_THRESHOLD = None

# 모터 출력(퍼센트, -100~100). 처음엔 느리고 안전하게.
FOLLOW_SPEED = 18          # 직진 라인추종 속도
TURN_SPEED = 16            # 제자리 회전 기준 속도(아래 좌/우/U턴이 기본값으로 상속)

# 비례 제어 게인. 선에서 벗어난 정도에 비례해 조향한다.
# 반응이 굼뜨면 키우고, 좌우로 떨면(진동) 줄인다.
KP = 0.75
# 좌/우 센서 중 한쪽만 선을 볼 때 더해 주는 강한 보정량(코너 빠른 복귀용).
SIDE_CORRECTION = 14

# 좌우 모터 편차 보정(곱셈 트림, 1.0=보정 없음). 같은 명령에도 좌우가 다르게 돌면 조정.
#   증상: 직진인데 한쪽으로 흐른다 → 빠른 쪽 트림을 살짝 낮추거나 느린 쪽을 높인다.
#   증상: 90도 회전이 항상 부족/과다 → 회전은 ignore/min 시간으로, 직진 쏠림은 트림으로 분리해 잡는다.
LEFT_MOTOR_TRIM = 1.0
RIGHT_MOTOR_TRIM = 1.0

# 메인 루프 주기(초). 작을수록 센서를 자주 읽는다(약 66Hz).
LOOP_DELAY = 0.015

# =============================================================================
# 3. 이벤트(분기/잎) 확정 — 패턴 단위 debounce
# =============================================================================
# 도착 이벤트는 "같은 bits 패턴이 연속으로 N번" 보일 때만 확정한다. 그래서
# 110->111->101 같은 회전·통과 중 흔들림은 누적되지 않고(패턴이 바뀌면 리셋),
# 진짜로 분기 중심에 자리잡아 패턴이 안정됐을 때만 멈춘다.
#   증상: 흔들림에 분기를 너무 일찍 확정 → 값 ↑. 분기에서 잘 못 멈춤 → 값 ↓.
JUNCTION_CONFIRM_SAMPLES = 4
# 잎(000)은 분기보다 "더 보수적으로" 확정한다(아래 5절 라인 유실 복구와 함께).
#   잎을 너무 쉽게 인정하면, 선을 잠깐 놓친 것을 막다른 길로 오인한다. 반드시 >= JUNCTION.
LEAF_CONFIRM_SAMPLES = 8
# 도착 이벤트 확정 모드(흔들림 흡수 방식):
#   "pattern" — 같은 bits 가 연속 동일해야 확정. 110->111->101 흔들림에 강하다(기본, 권장).
#   "kind"    — 같은 종류(둘 다 분기면 110<->111 이 섞여도)면 누적. 분기 모양이 빠르게
#               떨려 패턴이 좀체 안정 안 되는 코스에서 확정이 빨라 중심을 덜 지나친다.
#   증상: 분기 확정이 너무 늦어 중심을 지나친다 → "kind". 노이즈에 일찍 확정된다 → "pattern".
EVENT_DEBOUNCE_MODE = "pattern"
# 분기 확정 후 "분기 중심"까지 더 전진하는 시간(초). 0이면 즉시 정지.
#   증상: 분기를 감지했는데 아직 중심 전이라 회전이 어긋남 → 값 ↑. 너무 깊이 들어가면 ↓.
JUNCTION_CENTERING_SECONDS = 0.0

# =============================================================================
# 4. 라인 유실 복구 (000 을 무조건 막다른 길로 보지 않는다)
# =============================================================================
# 000(선 끊김)을 보면 곧장 leaf 로 확정하지 말고, 아주 잠깐만 저속 전진하며
# 선을 다시 잡는지 본다. 이 시간이 지나면 더 전진하지 않고 '멈춰서 제자리 재샘플'
# 한다(solver). 그래도 LEAF_CONFIRM_SAMPLES 만큼 000 이 지속되면 막다른 길로 확정.
#   증상: 선 살짝 놓친 걸 막다른 길로 오판 → 시간/샘플 ↑. 진짜 막다른 길에서 너무 굼뜸 → ↓.
#   기본값은 매우 보수적으로(짧게·느리게) 둬 막다른 벽 충돌을 피한다.
LOST_LINE_RECOVERY_SECONDS = 0.15
# 복구 전진 속도(FOLLOW_SPEED 보다 확실히 느리게). 막다른 길 벽에 박지 않게.
LOST_LINE_RECOVERY_SPEED = 10

# =============================================================================
# 5. 회전 — 좌/우/U턴 따로 튜닝 (라인 재포착 방식, 자이로 불필요)
# =============================================================================
# 실제 로봇은 마찰·바닥재질·배터리 전압·바퀴 상태·좌우 모터 편차 때문에
# 좌회전 90, 우회전 90, U턴 180 이 코드상 각도와 다르게 돈다. 그래서 셋을 따로 둔다.
#
# 각 회전의 동작 순서:
#   (1) IGNORE 초 동안은 센서를 아예 보지 않고 회전만 한다(출발 선 잔상 무시).
#   (2) MIN 초까지는 무조건 더 회전한다(센서가 다닥다닥 붙어 출발 선 모서리를
#       곧장 다시 잡아 회전이 덜 되는 것을 방지하는 "최소 회전 시간" = 엔코더 각도 대용).
#   (3) REQUIRE_LINE_CLEAR 이면 "선을 한 번 벗어났다가" 다시 잡을 때만 정지로 인정.
#   (4) 중앙센서가 선을 다시 잡으면 정지 → POST_TURN settle.
# 반드시 IGNORE < MIN < TIMEOUT.
LEFT_TURN_SPEED = TURN_SPEED
RIGHT_TURN_SPEED = TURN_SPEED
UTURN_SPEED = TURN_SPEED

# 회전 시작 직후 출발 선을 "다시 잡았다"고 오인하지 않도록 무시하는 시간(초).
#   증상: 회전을 시작하자마자 멈춰 버림 → 값 ↑.
LEFT_TURN_IGNORE_SECONDS = 0.18
RIGHT_TURN_IGNORE_SECONDS = 0.18
# U턴은 180도라 더 오래 무시한다(첫 선을 지나치고 두 번째 선에서 멈추기 위함).
UTURN_IGNORE_SECONDS = 0.55

# 최소 회전 시간(초). 이 시간 전에는 선을 잡아도 정지하지 않는다(과소회전 방지).
#   증상: 90도가 자꾸 덜 돌아 비스듬히 선다 → 값 ↑(부족분만큼). 과다회전이면 ↓.
LEFT_TURN_MIN_SECONDS = 0.30
RIGHT_TURN_MIN_SECONDS = 0.30
UTURN_MIN_SECONDS = 0.75

# 회전 후 중앙센서가 선을 다시 잡을 때까지 기다리는 최대 시간(초). 넘으면 오류.
LEFT_TURN_TIMEOUT_SECONDS = 3.0
RIGHT_TURN_TIMEOUT_SECONDS = 3.0
UTURN_TIMEOUT_SECONDS = 4.0

# 정지로 인정하기 전에 "선을 한 번 벗어났는지" 확인할지(다닥다닥 붙은 센서 대비).
# U턴은 특히 첫 선을 반드시 지나야 하므로 강하게 권장.
LEFT_TURN_REQUIRE_LINE_CLEAR = True
RIGHT_TURN_REQUIRE_LINE_CLEAR = True
UTURN_REQUIRE_LINE_CLEAR = True

# =============================================================================
# 6. 정지/관성 & 센서 안정화 딜레이
# =============================================================================
# stop(brake=True) 후에도 관성으로 살짝 밀리거나 튄다. 정지 후 잠깐 가만히 둔다.
#   증상: 정지 직후 읽은 패턴이 회전/분기 잔상이라 오판 → 값 ↑.
POST_STOP_SETTLE_SECONDS = 0.08
# 회전 완료 직후, 브레이크가 멎고 센서 위치가 안정될 때까지(기계적 settle).
POST_TURN_SETTLE_SECONDS = 0.10
# 회전 완료 직후, 센서 값이 안정될 때까지 추가로 기다리는 시간(읽기 안정화).
#   증상: 회전 직후 스캔이 110->111 처럼 튀어 분기 오판 → 값 ↑.
POST_TURN_SENSOR_SETTLE_SECONDS = 0.10
# 직진 nudge(S 토큰) 후 센서 안정화 시간.
POST_MOVE_SENSOR_SETTLE_SECONDS = 0.08
# 분기 출구를 본격 살피기(sense_exits) 전/후 센서 안정화 시간.
PEEK_SENSOR_SETTLE_SECONDS = 0.08

# 직진(S) 토큰에서 분기점을 지나 다음 라인에 올라타려 잠깐 전진하는 시간(초).
#   증상: 분기를 다 못 지나 다음 라인에 못 올라탐 → 값 ↑. 너무 멀리 가면 ↓.
STRAIGHT_NUDGE_SECONDS = 0.22

# 회전 직후 분기 위에 머물러 생기는 거짓 이벤트를 막으려 짧게 직진하는 시간(초).
# (예전엔 TURN_IGNORE_LINE_SECONDS 를 재사용했으나 의미가 섞여 분리했다.)
#   증상: 회전 후 같은 분기를 또 분기로 잡음 → 값 ↑. 다음 분기를 지나쳐 버리면 ↓.
CLEAR_JUNCTION_SECONDS = 0.18

# =============================================================================
# 7. 형태 peek (좌·우 동시 분기에서 직진 개통 확인)
# =============================================================================
# 1?1 분기는 모양이 헷갈리므로 살짝 전진해 중앙이 뚫렸는지(십자/T) 확인한다.
# 전진량·후진량·정지 딜레이·샘플 수를 각각 따로 둔다(원위치 복귀 정밀 튜닝).
PEEK_FORWARD_SECONDS = 0.20    # 앞으로 살짝
PEEK_BACK_SECONDS = 0.20       # 다시 제자리로 후진. 복귀가 덜 되면 ↑, 과하면 ↓.
PEEK_SETTLE_SECONDS = 0.10     # 전/후진 정지 후 안정화
# peek 판정은 중앙센서 한 번이 아니라 여러 번 읽어 다수결(majority)로 정한다.
#   증상: 111,101,111 같은 순간 흔들림에 직진 개통을 오판 → 값 ↑(홀수 권장).
PEEK_SAMPLES = 5

# =============================================================================
# 8. 노드 색 판정 — 중앙센서 "컬러 모드" (반사광과 별개)
# =============================================================================
# 지도가 없으므로 도착(7)과 일반 잎(체크포인트)을 위상만으로 구분할 수 없다.
# → 막다른 길에서 중앙센서를 컬러 모드로 읽어 "노드 색"으로 구분한다.
#
# 값은 ev3dev2 ColorSensor.color 의 정수 코드를 쓴다:
#   0=없음, 1=검정, 2=파랑, 3=초록, 4=노랑, 5=빨강, 6=흰색, 7=갈색
# 실제 코스의 마커 색에 맞춰 아래 3개를 바꿀 것. (셋은 서로 달라야 한다.)
START_COLOR = 4            # 시작(0) 마커 색 — RETURN 종료 판정에 사용
CHECKPOINT_COLOR = 2       # 노드(1~6) 마커 색 — 체크포인트(U턴 대상)
GOAL_COLOR = 5             # 도착(7) 마커 색 — EXPLORE 종료 판정에 사용
# 정상 주행에는 셋이 서로 달라야 위상 구분이 된다(validate_config 가 강제).
# 단, 색을 아직 못 정한 개발/벤치 단계에서 임시로 같게 두고 돌려보고 싶으면 True 로.
ALLOW_DUPLICATE_NODE_COLORS = False
# color 모드가 흔들릴 때를 대비해 같은 색이 연속 몇 번 보여야 인정할지.
COLOR_CONFIRM_SAMPLES = 3
# 반사광→컬러 모드 전환 직후 값이 튄다. 안정화 sleep(초)과 버리는 더미 읽기 횟수.
#   증상: 색을 자꾸 0/엉뚱한 값으로 읽음 → settle ↑, dummy ↑.
COLOR_MODE_SETTLE_SECONDS = 0.12
COLOR_DUMMY_READS = 2
# 컬러 모드에서 반사광 모드로 되돌아온 뒤에도 라인추종 재개 전 짧게 안정화.
COLOR_MODE_RESTORE_SETTLE_SECONDS = 0.08

# =============================================================================
# 9. 초음파(장애물/물체) + 그립&리프트
# =============================================================================
# 주행 중 전방 거리가 이 값(cm)보다 가까우면 "물체 발견"으로 보고 집는다.
OBSTACLE_DISTANCE_CM = 8.0
# 벽/분기구조/로봇 부품 때문에 비현실적으로 작은 값이 나올 수 있다. 이 값 미만은 무시(노이즈).
OBSTACLE_MIN_VALID_CM = 1.0
# 초음파 거리도 한 번 튀는 값에 속지 않도록 연속 확인 횟수.
OBSTACLE_CONFIRM_SAMPLES = 3
# 물체 감지를 EXPLORE 주행 중에만 허용할지(RETURN 때는 이미 내려놓았으므로 오탐 방지).
OBJECT_DETECT_ON_EXPLORE_ONLY = True
# 미디엄 모터로 집게를 닫는 각도/속도(도, 퍼센트). 코스 물체에 맞춰 조정.
GRIP_CLOSE_DEGREES = 110
GRIP_SPEED = 40
# 집은 뒤 살짝 들어 올려 주행/센서를 방해하지 않게 하는 각도(0이면 리프트 안 함).
LIFT_DEGREES = 0
# 집게/내려놓기 동작 후 로봇 자세가 흔들린다. 라인추종 재개 전 안정화 시간(초).
#   증상: 집은 직후 라인을 놓치거나 휘청 → 값 ↑.
POST_GRIP_SETTLE_SECONDS = 0.20
POST_RELEASE_SETTLE_SECONDS = 0.20

# =============================================================================
# 10. 디버그/로그 (실기 튜닝용)
# =============================================================================
# raw 센서값·bits 패턴·이벤트 확정·turn 토큰·색·거리를 출력한다.
# 너무 잦은 로그가 주행 타이밍을 망치지 않도록 주기를 제한한다.
DEBUG_SENSOR_LOG = False           # 매 루프 raw/bits 출력(주기 제한 적용)
DEBUG_TURNS = False                # 회전 시작/종료/소요시간
DEBUG_EVENTS = False               # 분기/잎 이벤트 확정 시 패턴
DEBUG_COLOR = False                # 컬러 raw/색번호
DEBUG_DISTANCE = False             # 초음파 거리
DEBUG_LOG_INTERVAL_SECONDS = 0.25  # DEBUG_SENSOR_LOG/DISTANCE 최소 출력 간격

# =============================================================================
# 11. 운용
# =============================================================================
# 각 단계(EXPLORE/RETURN) 시작 전에 브릭 가운데 버튼을 기다릴지.
WAIT_FOR_BUTTON = True

# =============================================================================
# 12. 폴백 플랜 (--plan 모드 전용)
# =============================================================================
# 자율 주행이 실패할 때 쓰는, 시뮬레이터에서 검증된 고정 경로.
# 1=좌(L), 2=직(S), 3=우(R), 4=U턴. 자율 모드에서는 사용하지 않는다.
EXPLORE_PLAN = [1, 4, 1, 1, 4, 2, 2, 4, 1, 4, 2, 1, 4, 1, 4, 3, 1, 4, 1]


# =============================================================================
# 13. 설정 검증 — 말이 안 되는 값을 조기에 잡는다
# =============================================================================
def _threshold(value):
    """센서별 threshold 가 None 이면 공용 LINE_THRESHOLD 로 폴백."""
    return LINE_THRESHOLD if value is None else value


def line_thresholds():
    """(좌, 중, 우) threshold 튜플. 코드는 이걸 통해 bits 를 만든다."""
    return (
        _threshold(LEFT_LINE_THRESHOLD),
        _threshold(CENTER_LINE_THRESHOLD),
        _threshold(RIGHT_LINE_THRESHOLD),
    )


def validate_config():
    """값이 음수이거나 모순(timeout<=ignore 등)이면 ValueError 로 한 번에 보고."""
    errors = []

    def need(cond, msg):
        if not cond:
            errors.append(msg)

    # 속도 범위 (-100~100), 주행/회전은 양수여야 전진/회전한다.
    for name, v in (("FOLLOW_SPEED", FOLLOW_SPEED), ("TURN_SPEED", TURN_SPEED),
                    ("LEFT_TURN_SPEED", LEFT_TURN_SPEED),
                    ("RIGHT_TURN_SPEED", RIGHT_TURN_SPEED),
                    ("UTURN_SPEED", UTURN_SPEED),
                    ("LOST_LINE_RECOVERY_SPEED", LOST_LINE_RECOVERY_SPEED)):
        need(0 < v <= 100, "{} 는 0 초과 100 이하여야 함 (현재 {})".format(name, v))

    # 모터 트림은 양수.
    need(LEFT_MOTOR_TRIM > 0, "LEFT_MOTOR_TRIM 는 0 초과 (현재 {})".format(LEFT_MOTOR_TRIM))
    need(RIGHT_MOTOR_TRIM > 0, "RIGHT_MOTOR_TRIM 는 0 초과 (현재 {})".format(RIGHT_MOTOR_TRIM))

    # threshold 0~100.
    for name, v in (("LEFT", LEFT_LINE_THRESHOLD), ("CENTER", CENTER_LINE_THRESHOLD),
                    ("RIGHT", RIGHT_LINE_THRESHOLD)):
        tv = _threshold(v)
        need(0 <= tv <= 100, "{}_LINE_THRESHOLD 는 0~100 (현재 {})".format(name, tv))

    # debounce 샘플 수 >= 1, leaf 는 junction 보다 보수적(>=).
    need(JUNCTION_CONFIRM_SAMPLES >= 1, "JUNCTION_CONFIRM_SAMPLES >= 1 이어야 함")
    need(LEAF_CONFIRM_SAMPLES >= 1, "LEAF_CONFIRM_SAMPLES >= 1 이어야 함")
    need(LEAF_CONFIRM_SAMPLES >= JUNCTION_CONFIRM_SAMPLES,
         "LEAF_CONFIRM_SAMPLES 는 JUNCTION_CONFIRM_SAMPLES 이상이어야 함(잎이 더 보수적)")
    need(COLOR_CONFIRM_SAMPLES >= 1, "COLOR_CONFIRM_SAMPLES >= 1 이어야 함")
    need(PEEK_SAMPLES >= 1, "PEEK_SAMPLES >= 1 이어야 함")
    need(OBSTACLE_CONFIRM_SAMPLES >= 1, "OBSTACLE_CONFIRM_SAMPLES >= 1 이어야 함")
    need(COLOR_DUMMY_READS >= 0, "COLOR_DUMMY_READS >= 0 이어야 함")
    need(EVENT_DEBOUNCE_MODE in ("pattern", "kind"),
         'EVENT_DEBOUNCE_MODE 는 "pattern" 또는 "kind" (현재 {!r})'.format(EVENT_DEBOUNCE_MODE))

    # 회전 타이밍: 0 은 허용한다(예: min=0 으로 최소회전 끄기, ignore=0 으로 잔상무시 끄기).
    # 진짜 필요한 제약은 "재포착 가능 시점(ignore, min)이 timeout 안에 있어야" 한다는 것.
    for tag, ig, mn, to in (
            ("LEFT", LEFT_TURN_IGNORE_SECONDS, LEFT_TURN_MIN_SECONDS, LEFT_TURN_TIMEOUT_SECONDS),
            ("RIGHT", RIGHT_TURN_IGNORE_SECONDS, RIGHT_TURN_MIN_SECONDS, RIGHT_TURN_TIMEOUT_SECONDS),
            ("UTURN", UTURN_IGNORE_SECONDS, UTURN_MIN_SECONDS, UTURN_TIMEOUT_SECONDS)):
        need(ig >= 0, "{}_TURN_IGNORE_SECONDS >= 0 (현재 {})".format(tag, ig))
        need(mn >= 0, "{}_TURN_MIN_SECONDS >= 0 (현재 {})".format(tag, mn))
        need(ig < to, "{}: IGNORE({}) < TIMEOUT({}) 이어야 함(아니면 영영 못 멈춤)".format(tag, ig, to))
        need(mn < to, "{}: MIN({}) < TIMEOUT({}) 이어야 함(아니면 영영 못 멈춤)".format(tag, mn, to))

    # 음수가 되면 안 되는 시간들.
    nonneg = {
        "LOOP_DELAY": LOOP_DELAY, "STRAIGHT_NUDGE_SECONDS": STRAIGHT_NUDGE_SECONDS,
        "CLEAR_JUNCTION_SECONDS": CLEAR_JUNCTION_SECONDS,
        "JUNCTION_CENTERING_SECONDS": JUNCTION_CENTERING_SECONDS,
        "LOST_LINE_RECOVERY_SECONDS": LOST_LINE_RECOVERY_SECONDS,
        "POST_STOP_SETTLE_SECONDS": POST_STOP_SETTLE_SECONDS,
        "POST_TURN_SETTLE_SECONDS": POST_TURN_SETTLE_SECONDS,
        "POST_TURN_SENSOR_SETTLE_SECONDS": POST_TURN_SENSOR_SETTLE_SECONDS,
        "POST_MOVE_SENSOR_SETTLE_SECONDS": POST_MOVE_SENSOR_SETTLE_SECONDS,
        "PEEK_SENSOR_SETTLE_SECONDS": PEEK_SENSOR_SETTLE_SECONDS,
        "PEEK_FORWARD_SECONDS": PEEK_FORWARD_SECONDS, "PEEK_BACK_SECONDS": PEEK_BACK_SECONDS,
        "PEEK_SETTLE_SECONDS": PEEK_SETTLE_SECONDS,
        "COLOR_MODE_SETTLE_SECONDS": COLOR_MODE_SETTLE_SECONDS,
        "COLOR_MODE_RESTORE_SETTLE_SECONDS": COLOR_MODE_RESTORE_SETTLE_SECONDS,
        "POST_GRIP_SETTLE_SECONDS": POST_GRIP_SETTLE_SECONDS,
        "POST_RELEASE_SETTLE_SECONDS": POST_RELEASE_SETTLE_SECONDS,
        "DEBUG_LOG_INTERVAL_SECONDS": DEBUG_LOG_INTERVAL_SECONDS,
    }
    for name, v in nonneg.items():
        need(v >= 0, "{} 는 음수일 수 없음 (현재 {})".format(name, v))

    # 초음파/그립.
    need(OBSTACLE_DISTANCE_CM > OBSTACLE_MIN_VALID_CM,
         "OBSTACLE_DISTANCE_CM 는 OBSTACLE_MIN_VALID_CM 보다 커야 함")
    need(GRIP_CLOSE_DEGREES >= 0, "GRIP_CLOSE_DEGREES >= 0")
    need(0 < GRIP_SPEED <= 100, "GRIP_SPEED 는 0 초과 100 이하")

    # 노드 색: 0~7. 정상 주행엔 셋이 서로 달라야 위상 구분이 가능(개발용 우회 플래그 제공).
    for name, v in (("START_COLOR", START_COLOR), ("CHECKPOINT_COLOR", CHECKPOINT_COLOR),
                    ("GOAL_COLOR", GOAL_COLOR)):
        need(0 <= v <= 7, "{} 는 0~7 (현재 {})".format(name, v))
    if not ALLOW_DUPLICATE_NODE_COLORS:
        need(len({START_COLOR, CHECKPOINT_COLOR, GOAL_COLOR}) == 3,
             "START/CHECKPOINT/GOAL_COLOR 는 서로 달라야 함"
             " (개발 중 임시로 같게 두려면 ALLOW_DUPLICATE_NODE_COLORS=True)")

    if errors:
        raise ValueError("config.py 검증 실패:\n  - " + "\n  - ".join(errors))
    return True
