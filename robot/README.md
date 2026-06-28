# EV3 Robot Runner

ev3dev-stretch에서 실행할 Python 코드입니다. **기본은 지도를 모르는 자율 주행**입니다:
로봇이 컬러센서 3개·초음파 1개로 직접 분기를 판단하며 모든 노드를 지나 도착(7)까지 갔다가
(EXPLORE), 그동안 쌓은 회전 기록을 거꾸로 재생해 출발(0)로 돌아옵니다(RETURN).
도중에 물체를 만나면 미디엄 모터로 집어 도착지점에 내려놓습니다.

자세한 알고리즘·코드 설명: **[ALGORITHM.md](ALGORITHM.md)**

## Wiring

| 역할 | 포트 |
|---|---|
| 주행 좌 / 우 모터 | `outA` / `outB` |
| 그립&리프트 모터 | `outC` |
| 컬러센서 좌 / 중 / 우 | `in1` / `in2` / `in3` |
| 초음파 센서 | `in4` |

포트·속도·임계값·색 코드 등 모든 튜닝 값은 [config.py](config.py) 에 주석과 함께 모여 있습니다.
실제 로봇은 마찰·배터리·바퀴·좌우 모터 편차 때문에 코드상 각도/타이밍과 다르게 움직이므로,
**좌/우/U턴 회전 보정값과 좌/중/우 센서 threshold 를 각각 따로** 둘 수 있게 분리해 두었습니다.
실기 튜닝 순서와 "증상 → 어떤 값" 표는 [ALGORITHM.md](ALGORITHM.md) 8장을 참고하세요.
시작 시 `config.validate_config()` 가 모순된 값(예: `timeout <= ignore`)을 미리 잡습니다.

## Files

- `main.py` — 진입점(CLI)
- `solver.py` — 알고리즘(`MazeSolver`) + 주행 계층(`Ev3Motion`)
- `hardware.py` — ev3dev2 모터·센서 입출력(`Ev3Hardware`)
- `config.py` — 튜닝 값
- `calibrate.py` — 브릭 버튼으로 쓰는 센서/동작 캘리브레이터
- `tests/sim_maze.py` — ev3dev 없이 알고리즘을 자동 검증하는 하니스

## Local Check

```bash
python3 robot/main.py --dry-run        # 폴백 플랜/역재생 출력
python3 -m py_compile robot/*.py       # 문법 점검
python3 robot/tests/sim_maze.py        # 자율 알고리즘 자동 검증(모두 PASS여야 함)
```

## Calibrator

EV3 브릭 위에서 센서값을 보고, 저장된 `config.py` 값으로 회전/라인추종/그리퍼 동작을
하나씩 테스트하는 도구입니다. 기본 실행은 읽기 전용이라 `config.py`를 바꾸지 않습니다.

```bash
PYTHONDONTWRITEBYTECODE=1 python3 robot/calibrate.py
```

메뉴 조작:

- LEFT / RIGHT: 메뉴 그룹 이동 (`MEASURE`, `ACTION`, `DEBUG`, `ADJUST`, `SYSTEM`)
- UP / DOWN: 그룹 안의 항목 선택 또는 값/태그 변경
- ENTER: 선택, 스냅샷 기록, 동작 1회 실행
- BACK: 즉시 모터 정지 후 이전 메뉴로 복귀

자주 쓰는 흐름:

- `MEASURE > Sensor raw`: 흰 바닥과 검은 선을 각각 찍어 threshold 후보 확인
- `MEASURE > Sensor bits`: 현재 threshold 기준 `000~111` 판정 확인
- `ACTION > Left/Right/U-turn`: 저장된 회전값으로 실제 회전 1회 테스트
- `DEBUG > Line only`: 분기 판단 없이 라인추종만 확인
- `DEBUG > Grip jog`: 그리퍼를 누르는 동안만 천천히 움직이며 안전 각도 확인
- `SYSTEM > Log path`: 현재 세션 로그 파일 위치 확인

측정과 테스트 결과는 `robot/logs/tune_*.log`에 남습니다. 튜닝값을 브릭에서 바로 저장하려면
명시적으로 `--write`를 붙여 실행합니다.

```bash
PYTHONDONTWRITEBYTECODE=1 python3 robot/calibrate.py --write
```

`--write`에서도 동작 테스트가 자동 저장되지는 않습니다. `ADJUST`에서 값을 바꾸고 저장 확인 화면에서
ENTER를 한 번 더 눌러야 `config.py`에 기록됩니다. 저장 전 `robot/config.py.bak` 백업을 만들고,
저장 뒤에는 `config.validate_config()`를 다시 실행합니다.

전체 메뉴와 각 항목의 의미는 [CALIBRATOR_SPEC.md](CALIBRATOR_SPEC.md), 튜닝 순서는
[TUNING.md](TUNING.md)를 참고하세요.

## Run On EV3

```bash
python3 robot/main.py            # 기본: 자율 EXPLORE → RETURN
python3 robot/main.py --plan     # 폴백: 검증된 고정 플랜 재생(자율이 불안할 때)
python3 robot/main.py --explore-only
python3 robot/main.py --return-only
```

VS Code의 ev3dev-browser 확장에서 `EV3: run maze` 구성을 실행해도 됩니다.
브릭에서 가운데 버튼을 누르면 EXPLORE가 시작되고, 한 번 더 누르면 RETURN이 시작됩니다.
중지는 뒤로가기 버튼입니다.
