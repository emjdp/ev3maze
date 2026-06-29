# EV3 미로 로봇 튜닝 가이드

실기에서 보면서 쓰는 튜닝 노트. 모든 값은 `robot/config.py` 한 곳에 있다.
각 단계는 캘리브레이션 프로그램(`CALIBRATOR_SPEC.md`)의 동일 번호 스텝으로 측정한다.

## 0. 실행 전

```bash
PYTHONDONTWRITEBYTECODE=1 python3 robot/main.py --dry-run
PYTHONDONTWRITEBYTECODE=1 python3 robot/tests/sim_maze.py
```

둘 다 정상일 때 브릭에서 실행. 값 바꾼 뒤엔 `validate_config()`(main 시작 시 자동, 또는 캘리브레이터 스텝 12)로 모순값을 먼저 거른다.

지켜야 할 제약(위반 시 validate 실패):
- 회전마다 `IGNORE < TIMEOUT`, `MIN < TIMEOUT`
- `LEAF_CONFIRM_SAMPLES >= JUNCTION_CONFIRM_SAMPLES`
- `START_COLOR`, `CHECKPOINT_COLOR`, `GOAL_COLOR` 서로 다름 (임시 우회: `ALLOW_DUPLICATE_NODE_COLORS = True`)
- 속도값 `0 < v <= 100`, 트림 `> 0`, threshold `0~100`

---

## 1. 센서 흑/백 기준 — 캘리브레이터 스텝 1

좌/중/우 센서를 흰 바닥과 검은 라인에 각각 올리고 raw 측정.

| 센서 | 흰색 raw | 검정 raw | threshold(중간값) |
|------|---------|---------|------------------|
| 좌 |  |  |  |
| 중 |  |  |  |
| 우 |  |  |  |

값:
- `LINE_THRESHOLD` (기본 40)
- 특정 센서만 오판하면 `LEFT_/CENTER_/RIGHT_LINE_THRESHOLD` (기본 None=공용)

기준: threshold = (흰색 + 검정) / 2. 한 센서만 어긋나면 그 센서만 개별값.

---

## 2. 직진 라인추종 — 캘리브레이터 스텝 3

느리게 시작한다.

값: `FOLLOW_SPEED`(18), `KP`(0.75), `SIDE_CORRECTION`(14)

| 증상 | 조치 |
|------|------|
| 반응 느림 | `KP` ↑ |
| 좌우 진동 | `KP` ↓ |
| 코너 복귀 약함 | `SIDE_CORRECTION` ↑ |
| 전체 불안정 | `FOLLOW_SPEED` ↓ |

---

## 3. 직진 쏠림(트림) — 캘리브레이터 스텝 4

라인 없는 짧은 직진에서 한쪽으로 흐르는지 본다. (회전엔 트림 미적용이므로 회전 각도와 분리해서 잡는다.)

값: `LEFT_MOTOR_TRIM`(1.0), `RIGHT_MOTOR_TRIM`(1.0)

| 증상 | 조치 |
|------|------|
| 왼쪽으로 감김 | 왼쪽 트림 ↓ 또는 오른쪽 트림 ↑ |
| 오른쪽으로 감김 | 오른쪽 트림 ↓ 또는 왼쪽 트림 ↑ |

---

## 4. 좌/우 90도 회전 — 캘리브레이터 스텝 5·6

좌/우 따로 맞춘다. 순서: IGNORE(센서 무시) → MIN(최소 회전) → 라인 재포착 → 정지.

값(좌/우 각각): `*_TURN_SPEED`(16), `*_TURN_IGNORE_SECONDS`(0.18), `*_TURN_MIN_SECONDS`(0.30), `*_TURN_TIMEOUT_SECONDS`(3.0), `*_TURN_REQUIRE_LINE_CLEAR`(True)

| 증상 | 조치 |
|------|------|
| 너무 조금 돎 | `*_MIN_SECONDS` ↑ |
| 너무 많이 돎 | `*_MIN_SECONDS` ↓ |
| 시작하자마자 멈춤 | `*_IGNORE_SECONDS` ↑ |
| 출발 선 모서리를 곧장 다시 잡음 | `REQUIRE_LINE_CLEAR = True` 확인 |
| 선 못 잡고 timeout | 속도 ↓ 또는 `*_TIMEOUT_SECONDS` ↑ |

---

## 5. U턴 — 캘리브레이터 스텝 7

좌/우 회전과 따로. 첫 선을 지나치고 두 번째 선에서 멈춰야 하므로 IGNORE를 길게.

값: `UTURN_SPEED`(16), `UTURN_IGNORE_SECONDS`(0.55), `UTURN_MIN_SECONDS`(0.75), `UTURN_TIMEOUT_SECONDS`(4.0), `UTURN_REQUIRE_LINE_CLEAR`(True)

| 증상 | 조치 |
|------|------|
| 첫 선에서 멈춤 | `UTURN_IGNORE_SECONDS` 또는 `UTURN_MIN_SECONDS` ↑ |
| 너무 많이 돎 | `UTURN_MIN_SECONDS` ↓ |
| 선 다시 못 잡음 | `UTURN_TIMEOUT_SECONDS` ↑ 또는 속도 ↓ |

---

## 6. 회전/정지 후 안정화 — 캘리브레이터 스텝 5~7 관찰 중

회전·정지 직후 잔상 패턴을 읽으면 조정.

값: `POST_TURN_SETTLE_SECONDS`(0.10), `POST_TURN_SENSOR_SETTLE_SECONDS`(0.10), `POST_STOP_SETTLE_SECONDS`(0.08), `POST_MOVE_SENSOR_SETTLE_SECONDS`(0.08)

| 증상 | 조치 |
|------|------|
| 회전 직후 분기 오판 | `POST_TURN_*_SETTLE` ↑ |
| 정지 후 밀림/튐 | `POST_STOP_SETTLE_SECONDS` ↑ |

---

## 7. 분기 감지 — 캘리브레이터 스텝 8

분기 확정이 빠른지/늦은지 본다.

값: `EVENT_DEBOUNCE_MODE`("pattern"), `JUNCTION_CONFIRM_SAMPLES`(4), `PRE_LEFT/RIGHT_TURN_FORWARD_SECONDS`(0.15), `CLEAR_JUNCTION_SECONDS`(0.18), `STRAIGHT_NUDGE_SECONDS`(0.22)

메모: 출구 판단 전에 먼저 전진하는 center-nudge 는 현재 제거했다. 코스/로봇 조건이 바뀌면 나중에 다시 넣을 후보로 기억할 것.

| 증상 | 조치 |
|------|------|
| 노이즈에 일찍 확정 | `EVENT_DEBOUNCE_MODE = "pattern"`, `JUNCTION_CONFIRM_SAMPLES` ↑ |
| 분기 지나침/늦게 확정 | `"kind"` 또는 `JUNCTION_CONFIRM_SAMPLES` ↓ |
| 출구 판단은 맞는데 90도 회전 시작 위치가 너무 앞쪽 | `PRE_LEFT/RIGHT_TURN_FORWARD_SECONDS` ↑ |
| 같은 분기 재감지 | `CLEAR_JUNCTION_SECONDS` ↑ |
| 분기 지나 다음 라인 못 올라탐 | `STRAIGHT_NUDGE_SECONDS` ↑ |

---

## 8. 막다른 길 / 선 유실 — 캘리브레이터 스텝 9

`000`(잎)이 자주 잘못 뜨면 조정. (잎은 분기보다 보수적으로 확정.)

값: `LEAF_CONFIRM_SAMPLES`(8), `LOST_LINE_RECOVERY_SECONDS`(0.15), `LOST_LINE_RECOVERY_SPEED`(10)

| 증상 | 조치 |
|------|------|
| 순간 유실을 막다른 길로 오판 | `LEAF_CONFIRM_SAMPLES`↑, `LOST_LINE_RECOVERY_SECONDS`↑ |
| 진짜 막다른 길에서 늦게 멈춤 | `LEAF_CONFIRM_SAMPLES` ↓ |
| 복구 중 벽으로 너무 감 | `LOST_LINE_RECOVERY_SECONDS`↓ 또는 `LOST_LINE_RECOVERY_SPEED`↓ |

---

## 9. Peek (좌·우 동시 분기 형태 판정) — 캘리브레이터 스텝 8

`1?1`에서 십자/T자 오판이면 조정. 전진 후 다수결로 직진 개통을 본다.

값: `PEEK_FORWARD_SECONDS`(0.20), `PEEK_BACK_SECONDS`(0.20), `PEEK_SETTLE_SECONDS`(0.10), `PEEK_SAMPLES`(5), `PEEK_SENSOR_SETTLE_SECONDS`(0.08)

| 증상 | 조치 |
|------|------|
| 십자를 T로 봄 | `PEEK_FORWARD_SECONDS` ↑ |
| T를 십자로 봄 | `PEEK_FORWARD_SECONDS` ↓ |
| peek 후 위치 틀어짐 | `PEEK_BACK_SECONDS` 조정(복귀 덜 되면 ↑) |
| 순간값에 흔들림 | `PEEK_SAMPLES` ↑(홀수) |

---

## 10. 색 판정 — 캘리브레이터 스텝 2·9

도착/출발/체크포인트 색 맞추기. 색 코드: 0없음 1검 2파 3초 4노 5빨 6흰 7갈.

값: `START_COLOR`(4), `CHECKPOINT_COLOR`(2), `GOAL_COLOR`(5), `COLOR_CONFIRM_SAMPLES`(3), `COLOR_MODE_SETTLE_SECONDS`(0.12), `COLOR_DUMMY_READS`(2), `COLOR_MODE_RESTORE_SETTLE_SECONDS`(0.08)

| 증상 | 조치 |
|------|------|
| 색이 튐 | `COLOR_MODE_SETTLE_SECONDS`↑, `COLOR_DUMMY_READS`↑, `COLOR_CONFIRM_SAMPLES`↑ |
| 계속 0/엉뚱한 색 | 마커 위치·센서 높이·색 번호 확인 |
| 임시로 같은 색 사용 | `ALLOW_DUPLICATE_NODE_COLORS = True` |

---

## 11. 물체 감지 / 집기 — 캘리브레이터 스텝 10·11

마지막에 맞춘다.

값: `OBSTACLE_DISTANCE_CM`(8.0), `OBSTACLE_MIN_VALID_CM`(1.0), `OBSTACLE_CONFIRM_SAMPLES`(3), `OBJECT_DETECT_ON_EXPLORE_ONLY`(True), `GRIP_CLOSE_DEGREES`(110), `GRIP_SPEED`(40), `LIFT_DEGREES`(0), `POST_GRIP_SETTLE_SECONDS`(0.20), `POST_RELEASE_SETTLE_SECONDS`(0.20)

| 증상 | 조치 |
|------|------|
| 너무 일찍 감지 | `OBSTACLE_DISTANCE_CM` ↓ |
| 못 감지 | `OBSTACLE_DISTANCE_CM` ↑ |
| 거리 튐 | `OBSTACLE_CONFIRM_SAMPLES` ↑ |
| 벽/부품을 물체로 오탐 | `OBSTACLE_MIN_VALID_CM` ↑ |
| 집은 뒤 라인 놓침 | `POST_GRIP_SETTLE_SECONDS` ↑ |

---

## 12. 추천 순서

1. `LINE_THRESHOLD`
2. `FOLLOW_SPEED`
3. `KP`, `SIDE_CORRECTION`
4. `LEFT_/RIGHT_MOTOR_TRIM`
5. 좌/우 90도 회전
6. U턴
7. 회전/정지 안정화
8. 분기 감지 + peek
9. 색 판정
10. 초음파/집기

---

## 13. 디버그

필요할 때만. 끝나면 끈다.

값: `DEBUG_SENSOR_LOG`, `DEBUG_TURNS`, `DEBUG_EVENTS`, `DEBUG_COLOR`, `DEBUG_DISTANCE`, `DEBUG_LOG_INTERVAL_SECONDS`(0.25)
