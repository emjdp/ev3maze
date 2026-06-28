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
- `tests/sim_maze.py` — ev3dev 없이 알고리즘을 자동 검증하는 하니스

## Local Check

```bash
python3 robot/main.py --dry-run        # 폴백 플랜/역재생 출력
python3 -m py_compile robot/*.py       # 문법 점검
python3 robot/tests/sim_maze.py        # 자율 알고리즘 자동 검증(모두 PASS여야 함)
```

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
