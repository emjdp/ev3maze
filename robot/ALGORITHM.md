# EV3 미로 로봇 — 코드 설명서 (팀 공용)

이 문서는 `robot/` 폴더의 **실제 Python 구현**을 팀원 모두가 이해하도록 풀어 쓴 것이다.
알고리즘의 플랫폼 무관 이론은 [../미로_알고리즘_구현명세.md](../미로_알고리즘_구현명세.md) 에 있고,
이 문서는 "그 이론을 ev3dev 코드로 어떻게 옮겼는가"에 집중한다.

> 한 줄 요약: **로봇은 미로 지도를 전혀 모른다.** 오직 컬러센서 3개·초음파 1개로
> 실시간 판단해 모든 노드를 지나 도착까지 갔다가(EXPLORE), 그동안 쌓은 회전 기록을
> 거꾸로 재생해 출발로 돌아온다(RETURN). 도중에 물체를 만나면 집어 도착지점에 내려놓는다.

---

## 1. 하드웨어 배선

| 역할 | 포트 | 종류 |
|---|---|---|
| 주행 좌 모터 | `outA` | Large Motor |
| 주행 우 모터 | `outB` | Large Motor |
| 그립&리프트 모터 | `outC` | Medium Motor |
| 컬러센서 좌 | `in1` | Color Sensor |
| 컬러센서 중 | `in2` | Color Sensor (주행=반사광 / 노드 판정=컬러) |
| 컬러센서 우 | `in3` | Color Sensor |
| 초음파 | `in4` | Ultrasonic Sensor |

포트·속도·임계값 등 **바꿀 만한 값은 전부 [config.py](config.py)** 에 주석과 함께 모았다.
코스에서 튜닝할 때는 이 파일만 고치면 된다.

---

## 2. 코드 구조 (파일별 역할)

알고리즘과 하드웨어를 계층으로 분리했다. 덕분에 ev3dev 없이도 알고리즘을 검증할 수 있다.

```
main.py      진입점(CLI). 모드 선택 후 단계 실행.
 └─ solver.py
     ├─ MazeSolver   "무엇을 할지" 결정하는 순수 알고리즘 (모터/센서 모름)
     └─ Ev3Motion    "어떻게 움직일지" 실행하는 주행 계층 (라인추종·회전·peek·집기)
         └─ hardware.py / Ev3Hardware   ev3dev2 모터·센서 입출력 캡슐화
 config.py   모든 튜닝 값
 tests/sim_maze.py   가짜 미로로 알고리즘을 자동 검증(아래 7장)
```

핵심은 **MazeSolver 가 Ev3Motion(=io) 의 높은 수준 명령만 호출**한다는 점이다:
`follow_to_node()`(다음 노드까지 가라), `sense_exits()`(출구를 살펴라),
`turn(token)`(이 방향으로 돌아라), `read_node_color()`(노드 색을 읽어라),
`deliver()`(물체를 내려놓아라). MazeSolver 는 모터를 모르므로, 같은 인터페이스의
가짜 주행 계층(SimMotion)을 끼우면 그대로 시뮬레이션된다.

---

## 3. 센서를 어떻게 쓰나 (3종 용도)

1. **반사광(주행)** — 평소 라인추종. 중앙센서가 검은 선을 보고 따라가고, 좌·우 센서는
   "분기가 나타나는지" 감시한다. `reflected_light_intensity` 를 `LINE_THRESHOLD`(기본 40)로
   잘라 0/1 비트 패턴을 만든다. 예: `010`=선 위(정상), `000`=막다른 길, `110`/`101`/`111`=분기.
2. **컬러(노드 판정)** — 막다른 길에서 **멈춘 뒤** 중앙센서를 잠깐 컬러 모드로 바꿔 노드 색을 읽는다.
   시작/체크포인트/도착의 색이 모두 다르다는 전제. **지도가 없으면 도착(7)과 일반 잎은 둘 다
   `000`(막다른 길)이라 위상만으로 구분이 안 된다 → 색으로 도착을 알아채 EXPLORE 를 끝낸다.**
   복귀 때는 시작 색을 보면 멈춘다. (`START_COLOR`/`CHECKPOINT_COLOR`/`GOAL_COLOR` in config)
3. **초음파(장애물/물체)** — 주행 루프 매 스텝에서 전방 거리를 본다. `OBSTACLE_DISTANCE_CM`
   보다 가까우면 물체로 보고 멈춰 미디엄 모터로 집는다(한 번만). 도착에서 내려놓는다.

---

## 4. EXPLORE — 지도 없는 자율 탐사

### 4-1. 분기 식별: "분기 로컬 프레임" (자이로 불필요)
지도·좌표·자이로가 없는데도 "이 분기에서 어느 출구를 이미 가봤나"를 어떻게 알까?

각 분기에 **처음 도착한 순간**을 기준으로 로컬 좌표를 박는다:

```
        직진(0°)
          │
왼쪽(270°)─┼─ 오른쪽(90°)
          │
        뒤=부모(180°)   ← 내가 들어온 길
```

- 출구를 0/90/180/270 네 각도로 기록한다(미로가 90° 격자라는 전제).
- 잎에서 U턴해 되돌아오면 "**방금 들어온 출구가 곧 내 뒤(180°)**"라는 사실로
  지금 바라보는 방향을 계산한다. 그러면 남은 출구를 좌/직/우로 정확히 다시 식별할 수 있다.
- 미로 엣지가 중간에 꺾여도(이 코스에 많음) **분기에서의 출구 방향**만 쓰므로 안전하다.
  전역 방위를 누적하지 않는 것이 핵심.

코드: `MazeSolver._explore_junction()` 의 `state = {각도: 상태}` 와 `facing` 변수.

### 4-2. 출구 선택 규칙
1. **형태 peek** — 좌·우가 동시에 보이는 분기(패턴 `1?1`)는 모양이 헷갈린다. 살짝 직진해
   중앙센서로 직진 개통을 확인한다. 뚫리면 **십자(D형)**, 막히면 **T자(B형)**. 재귀 구조상
   **분기당 딱 1회**만 일어난다. (`Ev3Motion.sense_exits()`)
2. **우선순위** — 십자면 "**직진부터**", 그 외는 "**좌선우선(좌>직>우)**"으로 미탐색 출구를 고른다.
   (`MazeSolver._pick_open()`)
3. **한 분기 우선** — 고른 출구가 **또 다른 분기**로 이어졌는데 **현재 분기에 아직 안 가본 다른
   출구가 남아 있으면**, 일단 U턴해 돌아와 잎부터 끝내고(그 출구는 '보류'), 나중에 내려간다.
   남은 미탐색 출구가 없으면 곧장 내려간다(불필요한 왕복 없음). 효과: 도착(7)이 있는 깊은
   쪽이 자연히 마지막이 되어 **전 노드를 지난 뒤 7**에 닿는다. (`_has_open_other`, `_descend`)
4. **잎(`000`)** — 색을 읽어 도착이면 종료, 아니면 체크포인트로 보고 U턴해 부모로 복귀.

### 4-3. 이 미로에서의 실제 동작(검증된 트레이스)
```
0 A 1 A B 2 B D 4 D 3 D E F E 5 E F 6 F 7
```
- **A**(좌+직): 좌선우선 → 1(잎) → 직진 출구 B 가 마지막 남은 분기라 곧장 내려감.
- **B**(좌+우, T형): peek로 직진 막힘 확인 → 좌(2,잎) → 우 출구 D 로 내려감.
- **D**(십자): peek로 직진 뚫림 확인 → **직진부터** 4(잎) → 3(잎) → 마지막 E 로 내려감.
- **E**(좌+직): 좌=F 로 가보니 분기 + 아직 5 가 남음 → **U턴 복귀(F 보류)** → 5(잎) → 보류한 F 로.
- **F**(좌+직): 6(잎) → 직진 7 = **도착 색 확인 → 종료**.

도착 직전 8/8 노드 방문, 단일 패스(사전 학습 0).

---

## 5. 기록 & RETURN — 어떻게 정확히 되돌아오나

- EXPLORE 중 **분기/잎에서 한 회전**을 숫자로 `path[]` 에 차곡차곡 쌓는다. (1=좌,2=직,3=우,4=U턴)
- RETURN 은 `path` 를 **거꾸로 + 좌우반전**해 재생한다(`return_plan()`). 갔던 길을 되짚으니
  순서는 뒤집히고, 진행 방향이 반대라 좌·우가 바뀐다. 직진·U턴은 그대로.
- 복귀 중에는 분기에서 **다시 판단·peek·스캔하지 않고** 기록대로만 회전한다 → 헤매지 않는다.
- 도착(7)은 막다른 길이라 먼저 한 번 U턴해 출발 쪽을 향한 뒤 재생을 시작하고,
  **시작 색**을 보면 멈춘다. (`MazeSolver.return_run()`)

> ⚠️ **RETURN 시작 가정**: `return_run()` 은 "도착에서 로봇이 막다른 길 끝을 향해
> 정확히 서 있다 → 먼저 U턴하면 되돌아갈 라인 위"라고 가정한다. 실기에서 도착 색을
> 읽은 시점에 로봇이 라인 끝에 덜/더 가 있거나 비스듬하면 U턴 후 라인을 못 잡을 수
> 있다. 이때는 `LEAF_CONFIRM_SAMPLES`/`LOST_LINE_RECOVERY_SECONDS` 로 도착 정렬을
> 다듬고, U턴은 `UTURN_*`(ignore/min/require_line_clear)로 "첫 선을 지나 두 번째
> 선에서 멈추게" 맞춘다.

---

## 6. 물체 집기 임무 흐름

```
주행(follow_to_node) 매 스텝:
  초음파 거리 < OBSTACLE_DISTANCE_CM 이고 아직 안 들었으면
     → 멈춤 → 미디엄 모터로 grip(필요 시 살짝 lift) → 들고 주행 계속
도착(7) 도달:
  들고 있으면 release → 도착지점에 내려놓기
```
관련 값: `OBSTACLE_DISTANCE_CM`, `GRIP_CLOSE_DEGREES`, `GRIP_SPEED`, `LIFT_DEGREES` (config).
가정: 물체 1개, 첫 감지 때 집고 도착에서 내려놓는다.

---

## 7. 실행 & 검증

### 오프라인(하드웨어 없이)
```bash
python3 robot/main.py --dry-run            # 폴백 플랜과 그 역재생 출력
python3 -m py_compile robot/*.py           # 문법 점검
python3 robot/tests/sim_maze.py            # 자율 알고리즘 자동 검증(아래 항목 전부 PASS여야 함)
```
`sim_maze.py` 는 가짜 미로 위에서 **실제 MazeSolver** 를 돌려:
EXPLORE 토큰이 검증값과 같은지, 노드 방문 순서가 맞는지, 8/8 방문·물체 집기·정확한 RETURN
까지 자동으로 확인한다. 알고리즘을 고쳤다면 **이 검사를 먼저 통과**시키고 브릭에 올린다.

### 브릭에서
```bash
python3 robot/main.py            # 기본: 자율 EXPLORE → RETURN
python3 robot/main.py --plan     # 폴백: 검증된 고정 플랜 재생(자율이 불안할 때)
python3 robot/main.py --explore-only   # 0 -> 7 만
python3 robot/main.py --return-only    # 7 -> 0 만(고정 플랜 기준)
```
가운데 버튼=단계 시작, 뒤로가기=중지.

---

## 8. config.py 튜닝 가이드

> 핵심 원칙: **실제 로봇은 코드상 각도/타이밍과 다르게 움직인다.** 마찰·바닥재질·
> 배터리 전압·바퀴 상태·좌우 모터 편차·센서 높이·주변광이 매번 다르기 때문이다.
> 그래서 하드웨어에서 조정할 값은 코드에 박지 않고 **전부 config.py** 로 뺐고,
> 좌/우/U턴, 좌/중/우 센서처럼 "현실에서 서로 다른" 것들은 따로 둘 수 있게 했다.
> 값마다 "어떤 증상일 때 올리고/내리는지" 주석을 달아 두었으니 함께 읽을 것.

### 8-1. 실기 튜닝 순서 (이 순서대로 잡으면 헤매지 않는다)
1. **센서 흑/백 raw 값 측정** — `DEBUG_SENSOR_LOG=True` 로 흰 바닥/검은 선의 raw 반사광을 각 센서별로 기록.
2. **라인 threshold 설정** — 흑·백 중간값으로 `LINE_THRESHOLD`. 센서마다 특성이 다르면 `LEFT/CENTER/RIGHT_LINE_THRESHOLD`.
3. **직진 라인추종 속도/KP** — `FOLLOW_SPEED`, `KP`, `SIDE_CORRECTION`. 한쪽으로 흐르면 `LEFT/RIGHT_MOTOR_TRIM`.
4. **좌/우 90도 회전** — `LEFT/RIGHT_TURN_*`(speed·ignore·min·timeout). 덜 돌면 min↑, 시작하자마자 멈추면 ignore↑.
5. **U턴** — `UTURN_*`. 첫 선에서 멈춰 버리면 `UTURN_REQUIRE_LINE_CLEAR=True` + `UTURN_MIN_SECONDS`↑.
6. **분기/peek 안정화** — `JUNCTION_CONFIRM_SAMPLES`, `CLEAR_JUNCTION_SECONDS`, `PEEK_*`, `POST_TURN_SENSOR_SETTLE_SECONDS`.
7. **컬러 판정** — `COLOR_MODE_SETTLE_SECONDS`, `COLOR_DUMMY_READS`, `START/CHECKPOINT/GOAL_COLOR`. `DEBUG_COLOR=True` 로 raw 색번호 확인.
8. **초음파/그립** — `OBSTACLE_DISTANCE_CM`, `OBSTACLE_MIN_VALID_CM`, `POST_GRIP_SETTLE_SECONDS`.

### 8-2. 새로 분리한 하드웨어 튜닝 값 (증상 → 어떤 값)
| 증상(실기에서 흔한 문제) | 손볼 config 값 |
|---|---|
| 직진인데 한쪽으로 흐른다 | `LEFT_MOTOR_TRIM` / `RIGHT_MOTOR_TRIM` |
| 특정 센서만 흑/백을 오판 | `LEFT/CENTER/RIGHT_LINE_THRESHOLD` |
| 좌/우 90도가 덜·더 돈다(좌우가 다르다) | `LEFT/RIGHT_TURN_MIN_SECONDS`, `..._IGNORE_SECONDS`, `..._SPEED` |
| 회전 시작하자마자 멈춘다(붙은 센서가 출발선 재포착) | `*_TURN_IGNORE_SECONDS`↑, `*_TURN_MIN_SECONDS`↑, `*_TURN_REQUIRE_LINE_CLEAR` |
| U턴이 첫 선에서 멈춘다 | `UTURN_REQUIRE_LINE_CLEAR=True`, `UTURN_MIN_SECONDS`↑ |
| 회전 직후 스캔이 110→111 잔상으로 분기 오판 | `POST_TURN_SETTLE_SECONDS`, `POST_TURN_SENSOR_SETTLE_SECONDS` |
| 정지 직후 관성으로 밀려 패턴 오판 | `POST_STOP_SETTLE_SECONDS` |
| 110→111→101 흔들림이 한 분기로 안 잡힘/오확정 | `JUNCTION_CONFIRM_SAMPLES` |
| 분기 모양이 떨려 확정이 늦고 중심을 지나침 | `EVENT_DEBOUNCE_MODE="kind"`(빠른 확정) ↔ `"pattern"`(노이즈 강함) |
| 직진(추종)은 트림 보정, 제자리 회전은 트림 미적용 | `LEFT/RIGHT_MOTOR_TRIM`(회전엔 자동 미적용) |
| 색 미정 개발 단계에서 임시로 색을 같게 두고 테스트 | `ALLOW_DUPLICATE_NODE_COLORS=True` |
| 선을 잠깐 놓친 걸 막다른 길로 오인 | `LEAF_CONFIRM_SAMPLES`↑, `LOST_LINE_RECOVERY_SECONDS`↑, `LOST_LINE_RECOVERY_SPEED` |
| 회전 후 같은 분기를 또 잡는다 | `CLEAR_JUNCTION_SECONDS` |
| 분기 감지했는데 아직 중심 전이라 회전이 어긋남 | `JUNCTION_CENTERING_SECONDS` |
| peek 직진 개통을 순간 노이즈로 오판 | `PEEK_SAMPLES`(홀수↑), `PEEK_SETTLE_SECONDS` |
| peek 후 원위치 복귀가 덜/과하다 | `PEEK_FORWARD_SECONDS` / `PEEK_BACK_SECONDS` |
| 컬러 모드 전환 직후 0/엉뚱한 색 | `COLOR_MODE_SETTLE_SECONDS`↑, `COLOR_DUMMY_READS`↑ |
| 집은 직후 휘청여 라인 놓침 | `POST_GRIP_SETTLE_SECONDS`, `POST_RELEASE_SETTLE_SECONDS` |
| 벽/부품을 물체로 오탐 | `OBSTACLE_MIN_VALID_CM`, `OBJECT_DETECT_ON_EXPLORE_ONLY` |

### 8-3. 자주 만지는 기본 값
| 값 | 의미 | 증상별 조정 |
|---|---|---|
| `LINE_THRESHOLD` | 반사광 흑/백 경계 | 흰 바닥을 선으로 오인하면 ↓, 선을 놓치면 ↑ (흑·백 중간값) |
| `FOLLOW_SPEED`/`TURN_SPEED` | 주행/회전 기준 속도 | 불안정하면 ↓ |
| `KP`/`SIDE_CORRECTION` | 라인추종 보정 세기 | 반응 느리면 ↑, 좌우로 떨면 ↓ |
| `START/CHECKPOINT/GOAL_COLOR` | 노드 색 코드(0~7) | 실제 마커 색으로(셋은 서로 달라야 함) |
| `OBSTACLE_DISTANCE_CM` | 물체 감지 거리 | 너무 일찍/늦게 집으면 조정 |
| `GRIP_CLOSE_DEGREES`/`LIFT_DEGREES` | 집게 동작량 | 물체 크기에 맞춰 |
| `DEBUG_*` | 튜닝용 로그 | raw/bits/turn/color/distance 출력. 주기 제한은 `DEBUG_LOG_INTERVAL_SECONDS` |

> 색 코드: 0=없음,1=검정,2=파랑,3=초록,4=노랑,5=빨강,6=흰색,7=갈색
> 흑/백 raw 값은 센서 높이·선 두께·주변광(ambient)에 따라 달라지므로, 코스가 바뀌면
> 2단계(threshold)부터 다시 잡는 게 안전하다.

### 8-4. 설정 검증
`config.validate_config()` 가 시작 시(main.py) 말이 안 되는 값을 잡는다: 속도 범위
초과, `timeout <= ignore`, debounce 샘플 < 1, `LEAF_CONFIRM_SAMPLES < JUNCTION_CONFIRM_SAMPLES`
(잎이 분기보다 덜 보수적), 노드 색 중복 등. 오프라인에서 `tests/sim_maze.py` 가 이
검증과 순수 판정 로직(흔들림 흡수)까지 자동 확인한다.
