# 캘리브레이션 / 튜닝 / 온브릭 디버거 명세서

브릭 위에서 버튼만으로 센서 측정, 개별 동작 테스트, 기능별 디버그 주행을 실행하고,
실측값을 화면·소리·로그로 남기는 온브릭 도구.

튜닝 대상값은 전부 `config.py`에 있다. 이 도구는 기본적으로 `config.py`의 **현재 저장값을
읽어서 그대로 행동**하고, 결과를 보여주고 기록한다. 즉 좌회전/우회전/U턴 버튼을 누르면
`LEFT_TURN_*`, `RIGHT_TURN_*`, `UTURN_*` 등 `config.py`에 저장된 값 기반으로 실제 모터가 돈다.
기본 실행에서는 `config.py`를 자동 수정하지 않는다.

## 1. 목적 / 범위

- 목적: `TUNING.md`의 각 단계를 키보드 없이 브릭 버튼만으로 측정·실행한다.
- 디버거 역할: 전체 자율주행 전에 센서, 라인트레이싱, 회전, 분기 판단, 물체 집기/내려놓기,
  단순 좌선우선 주행을 기능별로 쪼개서 따로 검증한다.
- 재사용: `hardware.Ev3Hardware`, `solver.Ev3Motion`의 기존 프리미티브를 우선 호출한다.
  새 코스 주행 로직을 만들지 않고, 디버그 모드는 기존 동작을 작은 단위로 조합한다.
- 비범위: 실제 대회용 전체 탐색/복귀 알고리즘 변경. 그건 `main.py`와 `solver.MazeSolver`가 담당한다.
- 파일: `robot/calibrate.py` (단독 실행). `python3 robot/calibrate.py`

## 2. 실행 / 저장 정책

```bash
PYTHONDONTWRITEBYTECODE=1 python3 robot/calibrate.py
```

- `config.validate_config()`를 시작 시 호출, 실패하면 오류 출력 후 종료(코드 2).
- `Ev3Hardware()` 생성(ev3dev2 필요). 하드웨어 미연결 환경이면 명확한 에러로 종료.
- `Ev3Motion(hw)` 래핑해 회전/추종/peek/그립 프리미티브 사용.
- 기본 실행은 읽기 전용이다. `config.py`를 자동으로 바꾸지 않는다.
- 실행 중 수동으로 `config.py`를 바꿨다면 프로그램을 재시작해야 새 값이 확실히 반영된다.
- 선택 기능으로만 `--write`를 둔다. 이 플래그가 있을 때만 ADJUST 화면에서 `config.py` 쓰기가 가능하다.

저장 옵션:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 robot/calibrate.py --write
```

- `--write`에서도 액션 버튼을 누른다고 바로 저장하지 않는다.
- ADJUST 항목에서 값을 바꾸고, 저장 확인 화면에서 ENTER를 한 번 더 눌러야 저장한다.
- 저장 전 `robot/config.py.bak`을 만들고, 로그에 이전값/새값을 남긴다.
- 저장 성공 후에는 메모리의 config 값을 갱신하거나 프로그램 재시작을 요구한다. 권장 구현은
  저장 직후 `importlib.reload(config)` 후 `config.validate_config()`를 다시 실행해, 바로 다음 ACTION이
  새 튜닝값으로 움직이게 하는 것이다.
- 사용 흐름은 `ADJUST에서 값 수정/저장 -> ACTION/DEBUG로 즉시 테스트 -> 다시 ADJUST`를 반복하는 방식이다.
- 저장 실패 시 모터를 멈추고 "수동 편집 필요"를 표시한 뒤 메뉴로 복귀한다.

## 3. 입력 — 브릭 버튼

`hardware.py`에 다음 헬퍼를 추가한다.

```python
def button_state(self):
    b = self.buttons
    return {"up": b.up, "down": b.down, "left": b.left,
            "right": b.right, "enter": b.enter, "back": b.backspace}
```

엣지(눌림→뗌) 검출로 1회 입력만 처리한다. 폴링 주기 0.05s.

| 버튼 | 메뉴 화면 | 스텝 실행 중 |
|------|----------|-------------|
| UP / DOWN | 항목 위/아래 선택 | ADJUST 값 증가/감소, MEASURE 태그 변경 |
| LEFT / RIGHT | 메뉴 그룹 이동 | ADJUST 대상/단위 변경, 디버그 옵션 토글 |
| ENTER | 선택 항목 진입 | MEASURE 스냅샷 기록, ACTION 1회 실행, DEBUG 시작/정지 |
| BACK | 이전 메뉴/종료 | 즉시 `hw.stop_all()` 후 스텝 중단 및 메뉴 복귀 |

안전: 모든 스텝 진입 시 모터 정지 상태에서 시작한다. BACK은 항상 주행 모터와 그립 모터를 모두 멈춘 뒤 복귀한다.
예외 발생 시에도 `finally`로 `hw.stop_all()`을 호출한다.

## 4. 출력 — 화면 + 소리 + 로그

### 화면

- `ev3dev2.display.Display`로 브릭 LCD에 출력하고 동일 내용을 `print()`로 콘솔에도 미러링한다.
- LCD 사용 불가 시 콘솔만 사용한다.
- 메뉴: 현재 선택 항목에 `>` 마커, 현재 그룹명, 위/아래 2~3개 항목 표시.
- 측정 화면: 라이브 수치 2~4줄(예: `L C R = 12 11 13`), 갱신 주기 0.1s.
- ACTION/DEBUG 화면: 실행 전 반드시 확인 화면을 둔다.
  - 예: `LEFT TURN`, `uses config.py`, `ENTER=run`, `BACK=cancel`
- 실행 후에는 결과를 1~2초 표시한다.

### 소리

모든 스텝은 실행마다 소리 피드백을 낸다. 소리는 실패해도 프로그램을 죽이지 않는다.

| 상황 | 소리 |
|------|------|
| 프로그램 시작 | 짧은 2회 beep |
| 메뉴 항목 진입 | 짧은 1회 beep |
| MEASURE 스냅샷 저장 | 높은 짧은 1회 beep |
| ACTION/DEBUG 시작 | 낮은 짧은 1회 beep |
| ACTION/DEBUG 정상 완료 | 짧은 2회 beep |
| BACK 중단 | 낮은 2회 beep |
| timeout/예외/검증 실패 | 긴 1회 tone 또는 낮은 3회 beep |
| 프로그램 종료 | 짧은 1회 beep |

`Ev3Hardware`에 `beep_ok()`, `beep_snapshot()`, `beep_start()`, `beep_abort()`,
`beep_error()` 같은 래퍼를 두되, ev3dev2 `Sound` 호출 실패는 무시한다.

### 로그

- 세션 로그: `robot/logs/tune_YYYYMMDD_HHMMSS.log` (UTF-8, append). 폴더 없으면 생성.
- 같은 프로그램 실행 안에서 여러 번 측정하면 **각 측정마다 별도 로그 라인**을 남긴다.
- ENTER 스냅샷, ACTION 반복 실행, DEBUG 중간 이벤트가 모두 누적된다.
- ADJUST 저장은 `CONFIG_WRITE` 로그로 남긴다. 값 이름, 이전값, 새값, 저장 성공 여부를 기록한다.
- 프로그램 종료 시 또는 SYSTEM의 최종값 기록 메뉴 실행 시, 현재 최종 튜닝값 전체를 `CONFIG_FINAL`
  블록으로 로그에 남긴다. 이 블록만 보면 마지막으로 확정한 `config.py` 상태를 복원할 수 있어야 한다.
- 각 스텝은 `run_id`와 `seq`를 가진다.
  - `run_id`: 스텝에 들어갈 때마다 1 증가
  - `seq`: 같은 스텝 안에서 스냅샷/반복 실행마다 1 증가
- 형식:

```text
ISO8601\t<GROUP>\t<STEP>\trun=<n> seq=<n> key=value ...\t<note>
```

예:

```text
2026-06-28T14:03:11\tMEASURE\tSENSOR_RAW\trun=3 seq=1 L=86 C=84 R=88 tag=white\tsnapshot
2026-06-28T14:03:18\tMEASURE\tSENSOR_RAW\trun=3 seq=2 L=18 C=16 R=19 tag=black\tsnapshot
2026-06-28T14:04:02\tACTION\tTURN_LEFT\trun=4 seq=1 elapsed=0.43 timeout=false\tok
2026-06-28T14:05:20\tDEBUG\tJUNCTION_111\trun=2 seq=7 raw=(12,11,13) bits=(1,1,1)\tconfirmed
2026-06-28T14:06:01\tADJUST\tCONFIG_WRITE\trun=9 seq=1 name=LEFT_TURN_MIN_SECONDS old=0.30 new=0.34\tok
2026-06-28T14:20:33\tSYSTEM\tCONFIG_FINAL\trun=0 seq=0 LEFT_TURN_MIN_SECONDS=0.34 RIGHT_TURN_MIN_SECONDS=0.31\tfinal
```

## 5. 메뉴 구조

최상위 메뉴는 너무 많은 기능이 한 화면에 섞이지 않도록 그룹으로 나눈다.

1. `MEASURE` — 센서값을 라이브로 보고 여러 번 스냅샷 저장
2. `ACTION` — 저장된 `config.py` 값으로 동작 1회 실행
3. `DEBUG` — 특정 기능만 반복/부분 실행
4. `ADJUST` — 선택적 값 조정 및 `--write` 저장
5. `SYSTEM` — 설정 검증, 로그 확인, 종료

## 6. MEASURE 메뉴

| # | 이름 | 호출 | 화면/로그 |
|---|------|------|-----------|
| M1 | 센서 반사광 raw | `hw.read_reflect()` 루프 | `L C R` 라이브. ENTER마다 스냅샷. UP/DOWN으로 태그 `white/black/line/free` 변경. 흰·검 둘 다 있으면 threshold 후보 표시 |
| M2 | 센서 bits 판정 | `hw.read_reflect()` + `bits_from_raw()` | 현재 `config.line_thresholds()` 기준 `000~111` 표시. ENTER마다 raw/bits 저장 |
| M3 | 중앙 노드 색 번호 | `hw.read_center_color()` 루프 | 색번호+이름(0없음..7갈) 라이브. ENTER마다 START/CHECKPOINT/GOAL 후보 태그로 저장 |
| M4 | 초음파 거리 | `hw.distance_cm()` 루프 | cm 라이브. `OBSTACLE_MIN_VALID_CM <= d < OBSTACLE_DISTANCE_CM`이면 `DETECT` 표시 |
| M5 | 버튼 상태 | `hw.button_state()` 루프 | 버튼 눌림 상태 확인. 버튼 배선/입력 디버그용 |

MEASURE 공통:

- ENTER를 여러 번 누르면 여러 로그 라인이 남는다.
- 스냅샷마다 소리를 낸다.
- BACK으로 종료해도 이전 스냅샷은 유지된다.

## 7. ACTION 메뉴

ACTION은 저장된 `config.py` 값 기반으로 실제 행동한다. 실행 전 확인 화면을 반드시 띄운다.

| # | 이름 | 호출 | 화면/결과 |
|---|------|------|-----------|
| A1 | 직진 라인추종 1회 | `motion.follow_to_node("CAL")` | 분기/잎 만날 때까지 추종. 도착 종류·bits·소요시간 로그 |
| A2 | 직진 쏠림(트림 ON) | `drive_straight(seconds, trim=True)` | `FOLLOW_SPEED`, `LEFT/RIGHT_MOTOR_TRIM` 적용. N초 직진 후 정지 |
| A3 | 직진 쏠림(트림 OFF) | `drive_straight(seconds, trim=False)` | 같은 시간 직진. 트림 효과 비교용 |
| A4 | 좌회전 90 | `motion.turn(LEFT)` | `LEFT_TURN_*` 값으로 회전 1회. 소요시간·timeout 로그 |
| A5 | 우회전 90 | `motion.turn(RIGHT)` | `RIGHT_TURN_*` 값으로 회전 1회. 소요시간·timeout 로그 |
| A6 | U턴 180 | `motion.turn(UTURN)` | `UTURN_*` 값으로 회전 1회. 소요시간·timeout 로그 |
| A7 | 직진 nudge | `motion.turn(STRAIGHT)` | `STRAIGHT_NUDGE_SECONDS` 만큼 전진. 분기 통과량 확인 |
| A8 | 분기 출구 감지/peek | `motion.sense_exits()` | 현재 위치에서 `{L,S,R}`와 `cross` 판정. peek 전·후진 포함 |
| A9 | 막다른 길 색 판정 | `motion.read_node_color()` | 정지 상태에서 중앙 색을 안정 판정 |
| A10 | 물체 집기+리프트 1회 | `safe_grip_close_lift()` | 반시계 방향으로 `GRIP_CLOSE_DEGREES` 후 `LIFT_DEGREES`. 실행 중 BACK/ENTER 즉시 정지 |
| A11 | 물체 내려놓기 1회 | `safe_grip_release()` | 시계 방향으로 `GRIP_CLOSE_DEGREES + LIFT_DEGREES` 만큼 복귀. 실행 중 BACK/ENTER 즉시 정지 |
| A12 | 물체 감지 후 집기 | `distance_cm()` 확인 후 `safe_grip_close_lift()` | 거리 조건 만족 시에만 집기. 만족하지 않으면 로그만 남김 |

ACTION 공통:

- ENTER 한 번당 실제 동작 1회, 로그 1개 이상.
- 같은 ACTION을 여러 번 반복하면 `seq`가 증가하며 모두 로그에 남는다.
- 회전 timeout(`RuntimeError`)은 잡아서 화면·소리·로그로 표시하고 메뉴로 복귀한다.
- ACTION 실행 중 BACK은 항상 즉시 중단이다. 그리퍼 ACTION에서는 ENTER도 "즉시 정지"로 처리한다.
- 그리퍼 ACTION은 blocking `on_for_degrees()`를 직접 쓰지 않고, 짧은 step/pulse 단위로 돌리며
  버튼을 0.02~0.05초마다 확인한다.

## 8. DEBUG 메뉴

DEBUG는 "전체 주행이 실패했을 때 어느 기능이 문제인지"를 좁히는 용도다.
대회용 알고리즘을 대체하지 않는다.

| # | 이름 | 목적 | 동작 |
|---|------|------|------|
| D1 | 라인트레이싱만 | 라인추종 제어 확인 | `follow_to_node("DEBUG_LINE")`와 달리 도착 후 종료하지 않고, BACK 전까지 라인만 계속 따라간다. raw/bits/속도 보정값을 주기적으로 로그 |
| D2 | 다음 노드까지만 | 분기/잎 확정 확인 | `motion.follow_to_node("DEBUG_NEXT")` 1회. 도착 종류, 마지막 bits, 소요시간 로그 |
| D3 | 111 분기만 판단 | 직진해서 십자/넓은 분기 감지 확인 | 라인을 따라 전진하다 `bits == (1,1,1)`이 `JUNCTION_CONFIRM_SAMPLES` 연속 나오면 정지. peek 없이 raw/bits만으로 판단 |
| D4 | 분기 패턴 스트림 | 센서 흔들림 확인 | 제자리 또는 저속 전진 중 raw/bits/event_kind/debounce 상태를 0.1s마다 화면·로그. BACK으로 종료 |
| D5 | peek만 반복 | 1?1 형태 판정 확인 | 현재 분기에서 ENTER마다 `motion.sense_exits()` 실행. 전진/후진 복귀량 확인 |
| D6 | 좌선우선 단순 완주 | 단순 라인트레이싱 완주 가능성 확인 | 지도/재귀/보류 없이 `L > S > R > U` 규칙만 반복. 잎에서는 색을 읽고 GOAL이면 종료, 아니면 U턴 |
| D7 | 회전 반복 테스트 | 회전값 안정성 확인 | 선택한 회전(L/R/U/S)을 ENTER마다 반복 실행. 성공/timeout/elapsed 분포 로그 |
| D8 | 그리퍼 수동 조그 | 그립 방향/토크 안전 확인 | LEFT=시계, RIGHT=반시계. 누르고 있는 동안만 저속 구동, 떼면 즉시 정지 |
| D9 | 물건 집기만 | 그립 기구 확인 | 주행 없이 ENTER마다 interruptible 집기+리프트 실행. ENTER/BACK 즉시 정지 |
| D10 | 물건 내려놓기만 | 내려놓기 기구 확인 | 주행 없이 ENTER마다 interruptible 내려놓기 실행. ENTER/BACK 즉시 정지 |
| D11 | 초음파 감지 스트림 | 물체 감지 기준 확인 | 거리와 detect 여부를 0.1s 표시. ENTER 스냅샷 저장 |
| D12 | 색 판정 반복 | 색 안정성 확인 | ENTER마다 `motion.read_node_color()` 실행. 연속 판정값 로그 |

### D1 라인트레이싱만

- 목적: `FOLLOW_SPEED`, `KP`, `SIDE_CORRECTION`, 센서 threshold가 맞는지 본다.
- 동작:
  - BACK 전까지 중앙선 추종만 한다.
  - 분기/잎을 만나도 자동으로 회전하거나 색을 읽지 않는다.
  - 안전을 위해 `DEBUG_LINE_MAX_SECONDS` 기본 20초를 둔다. 없으면 calibrate 내부 기본값 20초 사용.
  - 화면에는 raw/bits/left_speed/right_speed를 표시한다.

### D3 111 분기만 판단

- 목적: "직진해서 111 분기만 판단"을 따로 검증한다.
- 동작:
  - 시작 전 로봇을 라인 위에 둔다.
  - ENTER를 누르면 라인트레이싱으로 전진한다.
  - `bits == (1, 1, 1)`이 `JUNCTION_CONFIRM_SAMPLES` 연속 확인되면 정지하고 성공 로그.
  - `000`, `101`, `110`, `011` 등 다른 패턴은 화면과 로그에는 남기지만 성공으로 보지 않는다.
  - timeout 기본 10초. timeout 시 정지, 에러음, 마지막 raw/bits 로그.

### D6 좌선우선 단순 완주

- 목적: 복잡한 `MazeSolver` 없이, 센서/회전/라인추종만으로 코스가 대략 완주 가능한지 확인한다.
- 규칙:
  - 다음 노드까지 `motion.follow_to_node("DEBUG_LEFT")`.
  - 분기면 `motion.sense_exits()`로 출구를 보고 `L > S > R` 우선순위로 하나 선택한다.
  - 갈 곳이 없으면 U턴한다.
  - 잎이면 `motion.read_node_color()`를 실행한다.
    - `GOAL_COLOR`면 성공 종료.
    - 그 외 색이면 U턴 후 계속한다.
  - 지도, 재귀, 보류(deferred), RETURN path 기록, 물체 배달은 하지 않는다.
- 제한:
  - `DEBUG_LEFT_MAX_STEPS` 기본 80, `DEBUG_LEFT_MAX_SECONDS` 기본 180초.
  - 제한 초과 시 정지하고 실패가 아니라 "중단/한계 도달"로 로그.
  - 이 모드는 미로 구조에 따라 루프를 돌 수 있다. 그래서 항상 BACK 중단이 즉시 먹어야 한다.

### D8~D10 그리퍼 안전 튜닝

- 목적: 그리퍼가 오버토크로 부서지기 전에 사람이 즉시 멈추면서 적정 각도와 속도를 찾는다.
- 방향 약속:
  - 반시계 방향 = 집기 + 리프트 방향
  - 시계 방향 = 내려놓기/복귀 방향
- 구현 원칙:
  - 캘리브레이터의 그리퍼 테스트는 blocking `on_for_degrees()`를 직접 호출하지 않는다.
  - 작은 pulse 또는 motor `on()` + 짧은 sleep 루프로 돌리고, 매 0.02~0.05초마다 버튼을 확인한다.
  - 모터가 움직이는 중 BACK 또는 ENTER가 눌리면 즉시 `hw.stop_grip()` 후 중단한다.
  - 각 동작에는 최대 시간과 최대 각도 guard를 둔다. 기본 guard는 `GRIP_CLOSE_DEGREES + LIFT_DEGREES + 30도`
    또는 3초 중 먼저 도달하는 쪽으로 한다.
- D8 수동 조그:
  - RIGHT를 누르고 있는 동안만 반시계 방향 저속 구동.
  - LEFT를 누르고 있는 동안만 시계 방향 저속 구동.
  - 버튼을 떼면 즉시 정지.
  - 화면에 시작 위치 대비 motor position delta를 표시한다.
  - 모터가 정지한 상태에서 ENTER를 누르면 현재 delta를 스냅샷 저장한다.
  - UP/DOWN으로 태그를 `close/lift/release/free` 중 선택한다.
  - `close` 스냅샷은 `GRIP_CLOSE_DEGREES` 후보, `lift` 스냅샷은 `LIFT_DEGREES` 후보로 로그에 남긴다.
- D9 집기만:
  - 현재 저장된 `GRIP_CLOSE_DEGREES`, `LIFT_DEGREES`, `GRIP_SPEED`를 사용한다.
  - 반시계 방향으로 close 각도만큼 돌고, 이어서 lift 각도만큼 돈다.
  - 실행 중 ENTER/BACK 즉시 정지. 중단 시 실제 이동 delta를 로그에 남긴다.
- D10 내려놓기만:
  - 현재 저장된 `GRIP_CLOSE_DEGREES + LIFT_DEGREES`만큼 시계 방향으로 복귀한다.
  - 실행 중 ENTER/BACK 즉시 정지. 중단 시 실제 이동 delta를 로그에 남긴다.
- 튜닝 흐름:
  - D8에서 손으로 조그하며 안전한 close/lift delta를 찾는다.
  - ADJUST에서 `GRIP_CLOSE_DEGREES`, `LIFT_DEGREES`, `GRIP_SPEED`를 수정하고 저장한다.
  - D9/D10으로 저장값 기반 동작을 테스트한다.
  - 종료 시 최종값은 `CONFIG_FINAL`에 남는다.

## 9. ADJUST 메뉴

ADJUST는 선택적 확장이다. `--write`가 없으면 화면에서 후보값 계산과 임시 증감만 가능하고 저장은 막는다.
`--write`가 있으면 값을 저장하고 바로 ACTION/DEBUG로 테스트하는 반복 튜닝 흐름을 지원한다.

| 그룹 | 값 |
|------|----|
| 센서 threshold | `LINE_THRESHOLD`, `LEFT_LINE_THRESHOLD`, `CENTER_LINE_THRESHOLD`, `RIGHT_LINE_THRESHOLD` |
| 라인추종 | `FOLLOW_SPEED`, `KP`, `SIDE_CORRECTION` |
| 트림 | `LEFT_MOTOR_TRIM`, `RIGHT_MOTOR_TRIM` |
| 좌회전 | `LEFT_TURN_SPEED`, `LEFT_TURN_IGNORE_SECONDS`, `LEFT_TURN_MIN_SECONDS`, `LEFT_TURN_TIMEOUT_SECONDS`, `LEFT_TURN_REQUIRE_LINE_CLEAR` |
| 우회전 | `RIGHT_TURN_SPEED`, `RIGHT_TURN_IGNORE_SECONDS`, `RIGHT_TURN_MIN_SECONDS`, `RIGHT_TURN_TIMEOUT_SECONDS`, `RIGHT_TURN_REQUIRE_LINE_CLEAR` |
| U턴 | `UTURN_SPEED`, `UTURN_IGNORE_SECONDS`, `UTURN_MIN_SECONDS`, `UTURN_TIMEOUT_SECONDS`, `UTURN_REQUIRE_LINE_CLEAR` |
| 분기/peek | `JUNCTION_CONFIRM_SAMPLES`, `JUNCTION_CENTERING_SECONDS`, `STRAIGHT_NUDGE_SECONDS`, `CLEAR_JUNCTION_SECONDS`, `PEEK_*` |
| 잎/유실 | `LEAF_CONFIRM_SAMPLES`, `LOST_LINE_RECOVERY_SECONDS`, `LOST_LINE_RECOVERY_SPEED` |
| 색 | `START_COLOR`, `CHECKPOINT_COLOR`, `GOAL_COLOR`, `COLOR_*` |
| 초음파/그립 | `OBSTACLE_*`, `GRIP_*`, `LIFT_DEGREES`, `POST_GRIP_SETTLE_SECONDS`, `POST_RELEASE_SETTLE_SECONDS` |

저장 구현:

- `^<NAME>\s*=` 라인을 정규식으로 찾아 값만 교체하고 주석은 보존한다.
- 쓰기 전 새 값으로 `validate_config()`를 실행한다.
- 백업(`config.py.bak`) 후 기록한다.
- 저장 성공 시 `CONFIG_WRITE` 로그를 남긴다.
- 저장 성공 뒤 새 값을 즉시 테스트할 수 있게 `config`를 reload하거나, reload가 불안정하면 화면에
  "restart required"를 명확히 표시한다. 이 경우 ACTION은 이전 메모리 값을 쓸 수 있으므로 로그에도 남긴다.
- 실패하면 수기 편집으로 폴백하라고 안내한다.
- 메뉴에서 나가거나 프로그램을 종료할 때 `CONFIG_FINAL` 로그를 자동으로 남긴다.

## 10. SYSTEM 메뉴

| # | 이름 | 동작 |
|---|------|------|
| S1 | 설정 검증 | `config.validate_config()` 실행. 통과/실패 표시·로그 |
| S2 | 현재 핵심 설정 보기 | threshold, speed, trim, turn min/ignore/timeout, obstacle, grip 값을 화면에 요약 |
| S3 | 로그 파일명 보기 | 현재 세션 로그 경로 표시 |
| S4 | 소리 테스트 | 모든 beep 패턴을 차례로 실행 |
| S5 | 최종 튜닝값 로그 | 현재 `config.py`의 주요 튜닝값 전체를 `CONFIG_FINAL`로 즉시 기록 |
| S6 | 종료 | `CONFIG_FINAL` 자동 기록 후 `hw.stop_all()` 및 종료음 |

## 11. 신규로 필요한 코드

기존 프리미티브로 부족한 것만 최소 추가한다.

1. `Ev3Hardware.button_state()` — 3절.
2. 전체/개별 모터 정지 헬퍼:
   ```python
   def stop_drive(self): ...
   def stop_grip(self): ...
   def stop_all(self):
       self.stop_drive()
       self.stop_grip()
   ```
   `stop_all()`은 BACK, 예외, 프로그램 종료에서 항상 호출한다.
3. 소리 래퍼:
   ```python
   def beep_ok(self): ...
   def beep_snapshot(self): ...
   def beep_start(self): ...
   def beep_abort(self): ...
   def beep_error(self): ...
   ```
4. 직진 쏠림 테스트 헬퍼:
   ```python
   def drive_straight(motion, seconds, trim):
       motion._drive(config.FOLLOW_SPEED, config.FOLLOW_SPEED, trim=trim)
       # seconds 동안 BACK 감시
       motion._stop()
   ```
5. interruptible 그리퍼 헬퍼:
   ```python
   def run_grip_degrees(direction, degrees, speed, abort_buttons=("back", "enter")):
       # direction: "ccw" = 집기/리프트, "cw" = 내려놓기
       # blocking on_for_degrees 대신 짧게 구동하며 버튼/위치/timeout 감시
       # 중단/완료 시 hw.stop_grip()
       # 실제 이동 delta, aborted 여부, elapsed 로그
   ```
   그리퍼 캘리브레이션에서는 `hw.grip()`/`hw.release()`처럼 중간 중단이 어려운 래퍼를 직접 호출하지 않는다.
6. 그리퍼 수동 조그 헬퍼:
   - RIGHT press 유지 = 반시계 저속 구동, LEFT press 유지 = 시계 저속 구동.
   - 버튼 release, ENTER, BACK 중 하나라도 감지하면 즉시 `hw.stop_grip()`.
   - motor position delta를 화면과 로그에 남긴다.
7. 라인트레이싱만 디버그 헬퍼:
   - `motion._read()`와 `motion._follow_speeds()`를 사용한다.
   - `event_kind(bits)`는 표시/로그만 하고, 자동 정지 판정에는 쓰지 않는다.
8. 111 분기 판단 헬퍼:
   - `motion._read()`, `bits_from_raw()`, `JUNCTION_CONFIRM_SAMPLES`를 사용한다.
   - `bits == (1, 1, 1)`만 성공 조건으로 본다.
9. 단순 좌선우선 디버그 러너:
   - `follow_to_node`, `sense_exits`, `turn`, `read_node_color`만 조합한다.
   - `MazeSolver`의 path/deferred/return 로직은 사용하지 않는다.
10. 선택적 `config.py` 라이트백 — 기본 OFF, `--write`에서만 활성.

## 12. 동작 흐름(의사코드)

```text
validate_config()            # 실패 시 종료(2)
hw = Ev3Hardware()
motion = Ev3Motion(hw)
open session log
beep_startup()
group = MEASURE
sel = 0

loop:
    render_menu(group, sel)
    btn = wait_button_edge()
    if btn == left/right:
        group 이동
    elif btn == up/down:
        sel 이동
    elif btn == enter:
        run_id += 1
        beep_enter()
        try:
            confirm_if_motor_action()
            run_step(group, sel, run_id)
        except Aborted:
            log aborted
            beep_abort()
        except Exception as e:
            log error
            beep_error()
        finally:
            hw.stop_all()
    elif btn == back:
        break

hw.stop_all()
log_config_final()
beep_exit()
close log
```

MEASURE 루프:

```text
seq = 0
while True:
    render(live_values())     # 0.1s
    btn = poll_button_edge()
    if btn == enter:
        seq += 1
        log snapshot with run_id/seq
        beep_snapshot()
    if btn == back:
        return
```

ACTION 루프:

```text
show "uses config.py values"
ENTER=run, BACK=cancel
seq = 0
while True:
    btn = wait_button_edge()
    if btn == enter:
        seq += 1
        start_timer()
        run_action_once()
        log elapsed/result with run_id/seq
    if btn == back:
        return
```

## 13. 안전 규칙

- 어떤 경로로 빠져나가든 마지막에 `hw.stop_all()`.
- ACTION/DEBUG 중 주행 모터를 움직이는 항목은 실행 전 확인 화면을 둔다.
- 회전/추종/직진은 로봇이 코스 위 올바른 위치에 있을 때만 실행한다.
- 초음파/그립 단독 스텝은 주행 모터를 건드리지 않는다.
- BACK은 모든 루프에서 0.05초~0.1초마다 확인한다. 그리퍼 루프에서는 0.02~0.05초마다 확인한다.
- 그리퍼 동작 중 ENTER는 실행 버튼이 아니라 비상 정지 버튼으로 처리한다.
- 그리퍼 동작은 반드시 interruptible 방식으로 구현한다. `on_for_degrees()` 같은 blocking 호출은
  일반 주행 중 자동 집기에는 쓸 수 있어도, 캘리브레이터의 그리퍼 ACTION/DEBUG에는 쓰지 않는다.
- 그리퍼에는 최대 시간/최대 각도 guard를 둔다. guard 도달 시 `hw.stop_grip()`, 에러음, 로그, 메뉴 복귀.
- timeout/예외가 나면 모터 정지, 에러음, 로그 기록, 메뉴 복귀.
- DEBUG 장시간 주행에는 최대 시간/최대 step 제한을 둔다.
- 단순 좌선우선 모드는 루프 가능성이 있으므로 제한값과 BACK 중단을 필수로 둔다.

## 14. 수용 기준

- 키보드 없이 버튼만으로 MEASURE/ACTION/DEBUG/ADJUST/SYSTEM 메뉴 진입·실행·복귀 가능.
- 기본 실행에서는 `config.py`가 절대 수정되지 않는다.
- `--write`가 없을 때 저장 시도는 화면/로그에 "write disabled"로 남고 파일을 바꾸지 않는다.
- `--write`가 있을 때 ADJUST 저장은 이전값/새값을 `CONFIG_WRITE`로 기록하고, 저장 후 즉시 테스트 가능하거나
  "restart required"를 명확히 표시한다.
- 여러 번 측정하면 같은 세션 로그에 `run`/`seq`가 다른 여러 로그 라인이 남는다.
- 종료 시 마지막 튜닝값 전체가 `CONFIG_FINAL`로 로그에 남는다.
- 각 실행 시작/스냅샷/성공/중단/오류마다 소리가 난다.
- 좌회전/우회전/U턴 ACTION은 저장된 `config.py` 값 기반으로 실제 회전한다.
- MEASURE M1/M2/M3/M4에서 라이브 수치가 0.1s 주기로 갱신되고 ENTER 스냅샷이 로그에 남는다.
- DEBUG D1 라인트레이싱만, D3 111 분기만 판단, D6 좌선우선 단순 완주, D8 그리퍼 수동 조그,
  D9 집기만, D10 내려놓기만을 각각 단독 실행할 수 있다.
- 그리퍼 ACTION/DEBUG 중 BACK 또는 ENTER를 누르면 0.05초 이내에 그립 모터가 정지한다.
- D8 수동 조그에서 반시계/시계 방향을 직접 확인하고, close/lift/release delta 스냅샷을 로그에 남길 수 있다.
- BACK이 언제나 모터를 멈추고 메뉴로 돌아간다.
- 종료 후 `robot/logs/`에 이번 세션 로그 파일이 생성돼 있다.
