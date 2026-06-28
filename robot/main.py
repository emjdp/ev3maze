#!/usr/bin/env python3
"""EV3 미로 로봇 진입점 (ev3dev-stretch).

기본은 '지도를 모르는 자율 주행'이다: 로봇이 센서로 직접 분기를 판단하며
모든 노드를 지나 도착(7)까지 가고(EXPLORE), 그동안 쌓은 회전 기록을 거꾸로
재생해 출발(0)로 돌아온다(RETURN). 자율 주행이 불안할 때를 대비해, 시뮬레이터
에서 검증된 고정 경로를 그대로 재생하는 --plan 폴백 모드도 둔다.

  python3 robot/main.py                # 자율 EXPLORE → RETURN (기본)
  python3 robot/main.py --plan         # 검증된 고정 플랜 재생(폴백)
  python3 robot/main.py --explore-only # 0 -> 7 만
  python3 robot/main.py --return-only  # 7 -> 0 만(고정 플랜 기준)
  python3 robot/main.py --dry-run      # ev3dev 없이 플랜/역재생만 출력

알고리즘 설명: robot/ALGORITHM.md
"""

from __future__ import print_function

import argparse
import sys

import config
import solver


def print_dry_run():
    """하드웨어 없이, 폴백 고정 플랜과 그 역재생을 출력해 본다."""
    explore = list(config.EXPLORE_PLAN)
    ret = solver.return_plan(explore)
    names = lambda toks: " ".join(solver.TOKEN_NAME[t] for t in toks)
    print("EXPLORE (고정 플랜):", names(explore))
    print("RETURN  (역순+반전):", names(ret))
    print("EXPLORE tokens:", explore)
    print("RETURN  tokens:", ret)


def build_solver():
    """실제 하드웨어를 잡아 MazeSolver 를 만든다(ev3dev2 임포트는 여기서만 발생)."""
    from hardware import Ev3Hardware
    motion = solver.Ev3Motion(Ev3Hardware())
    return solver.MazeSolver(motion), motion


def run_autonomous(explore_only, return_only):
    brain, motion = build_solver()
    if return_only:
        motion.wait_for_start("RETURN")
        brain.return_run(explore_path=list(config.EXPLORE_PLAN))
        return
    motion.wait_for_start("EXPLORE")
    path = brain.explore()
    if explore_only:
        return
    motion.wait_for_start("RETURN")
    brain.return_run(path)


def run_plan(explore_only, return_only):
    brain, motion = build_solver()
    explore = list(config.EXPLORE_PLAN)
    ret = solver.return_plan(explore)
    if return_only:
        motion.wait_for_start("RETURN")
        brain.run_plan(ret, final_label="RETURN")
        return
    motion.wait_for_start("EXPLORE")
    brain.run_plan(explore, final_label="EXPLORE")
    if explore_only:
        return
    motion.wait_for_start("RETURN")
    brain.run_plan(ret, final_label="RETURN")


def parse_args(argv):
    p = argparse.ArgumentParser(description="EV3 미로 로봇")
    p.add_argument("--dry-run", action="store_true", help="ev3dev 없이 플랜만 출력")
    p.add_argument("--plan", action="store_true", help="검증된 고정 플랜 재생(폴백)")
    p.add_argument("--explore-only", action="store_true", help="0 -> 7 만")
    p.add_argument("--return-only", action="store_true", help="7 -> 0 만")
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])

    # 말이 안 되는 튜닝 값(음수, timeout<=ignore 등)은 주행 시작 전에 잡는다.
    try:
        config.validate_config()
    except ValueError as exc:
        print(exc, file=sys.stderr)
        return 2

    if args.dry_run:
        print_dry_run()
        return 0

    if args.explore_only and args.return_only:
        print("--explore-only 와 --return-only 는 함께 쓸 수 없습니다", file=sys.stderr)
        return 2

    runner = run_plan if args.plan else run_autonomous
    try:
        runner(args.explore_only, args.return_only)
    except solver.Aborted:
        print("중지됨")
        return 130
    except KeyboardInterrupt:
        print("중지됨")
        return 130
    return 0


if __name__ == "__main__":
    sys.exit(main())
