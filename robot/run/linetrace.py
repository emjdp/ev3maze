#!/usr/bin/env python3
"""Simple line-tracing test runner (ev3dev-stretch).

This is a deliberately small program for tuning line tracing in config.py
WITHOUT the full autonomous brain (no frame tracking, no deferred junctions,
no return run, no object gripping). Behaviour:

  - Start on the YELLOW node, follow the line.
  - At a junction: LEFT priority (Left > Straight > Right).
  - At a dead end: read the node color.
      * RED  (GOAL_COLOR)       -> arrived, stop (done).
      * BLUE (CHECKPOINT_COLOR) -> raise buzzer, U-turn, keep exploring.
      * other                   -> U-turn, keep exploring.
  - Live status is printed to the EV3 LCD the whole time (ASCII only).

All numbers come from config.py. The turn / color-read primitives are reused
from solver.Ev3Motion (the exact code those config values tune); only the
follow loop and the decision are kept simple and local so they are easy to read
and adjust here.

  python3 robot/run/linetrace.py
"""

from __future__ import print_function

import io
import sys
import time

# ev3dev runs in a C (ASCII) locale; wrap stdout/stderr so any stray non-ASCII
# from reused modules cannot crash us. The LCD itself we keep strictly ASCII.
for _name in ("stdout", "stderr"):
    _stream = getattr(sys, _name)
    _buffer = getattr(_stream, "buffer", None)
    if _buffer is not None:
        setattr(sys, _name, io.TextIOWrapper(
            _buffer, encoding="utf-8", errors="replace", line_buffering=True))

import config
from solver import (
    Ev3Motion, Aborted,
    bits_from_raw, event_kind, ArrivalDebouncer,
    LEAF, JUNCTION, LEFT, STRAIGHT, RIGHT, UTURN, TOKEN_NAME,
)
from hardware import Ev3Hardware


# =============================================================================
# LCD status display (ASCII only) — keeps the latest status lines on screen
# =============================================================================
class Screen(object):
    """Scrolling decision log on the EV3 LCD (newest at the bottom).

    event()       appends one line to the log and re-renders (use it whenever a
                  decision changes: junction turn, dead-end color, buzz, goal...).
    status_line() sets a single live line at the very bottom (raw/bits) without
                  polluting the log; safe to call often (throttled by caller).
    splash()      draws arbitrary lines, no log (wait / goal / abort screens).
    Falls back to stdout when not running on a brick.
    """

    LINE_H = 16        # pixels per line (default font ~8px tall)
    MAX_LOG = 6        # log lines kept on screen (top); +1 live line at bottom

    def __init__(self):
        self.disp = None
        try:
            from ev3dev2.display import Display
            self.disp = Display()
        except Exception:
            self.disp = None   # not on a brick: just print
        self.log = []
        self.status = ""

    def _draw(self, lines):
        if self.disp is None:
            return
        try:
            self.disp.clear()
            y = 0
            for ln in lines:
                self.disp.text_pixels(ln, clear_screen=False, x=2, y=y)
                y += self.LINE_H
            self.disp.update()
        except Exception:
            pass

    def _render(self):
        lines = list(self.log[-self.MAX_LOG:])
        # pad so the live status sits on a fixed bottom row
        while len(lines) < self.MAX_LOG:
            lines.append("")
        lines.append("> " + self.status if self.status else "")
        self._draw(lines)

    def event(self, text):
        self.log.append(text)
        print("LOG | " + text)
        self._render()

    def status_line(self, text):
        self.status = text
        if self.disp is None:
            return                 # don't spam the terminal with live readouts
        self._render()

    def splash(self, *lines):
        print("LCD | " + " | ".join(lines))
        self._draw(lines)


# =============================================================================
# Helpers (mirror solver's tuned follow math, no object pickup / no peek)
# =============================================================================
def follow_speeds(raw, bits):
    """Proportional steering, identical math to solver.Ev3Motion._follow_speeds."""
    correction = (raw[2] - raw[0]) * config.KP
    if bits[0] and not bits[2]:
        correction = abs(correction) + config.SIDE_CORRECTION
    elif bits[2] and not bits[0]:
        correction = -(abs(correction) + config.SIDE_CORRECTION)
    return config.FOLLOW_SPEED - correction, config.FOLLOW_SPEED + correction


def choose_turn(bits):
    """LEFT priority decision from the confirmed junction pattern: L > S > R."""
    if bits[0]:
        return LEFT
    if bits[1]:
        return STRAIGHT
    if bits[2]:
        return RIGHT
    return STRAIGHT


def buzz_up(hw):
    """Rising buzzer tone (checkpoint marker)."""
    for freq in (440, 660, 880):
        hw._tone(freq, 120)
        time.sleep(0.02)


# =============================================================================
# Simple follow loop — returns (kind, bits) when a junction/dead-end is confirmed
# =============================================================================
def seek_speeds_for(token):
    """회전 결정을 '유지'하며 선을 찾을 때, 그 방향으로 계속 돌 바퀴 속도(좌,우).

    좌/우/U턴은 제자리 피벗 방향 그대로 회전을 이어가 선을 찾는다(직진 탐색 아님).
    직진(STRAIGHT)은 회전이 아니므로 None → 일반 진입(앞으로 라인트레이싱).
    """
    if token == LEFT:
        return (-config.LEFT_TURN_SPEED, config.LEFT_TURN_SPEED)
    if token == RIGHT:
        return (config.RIGHT_TURN_SPEED, -config.RIGHT_TURN_SPEED)
    if token == UTURN:
        return (config.UTURN_SPEED, -config.UTURN_SPEED)
    return None


def follow_to_node(hw, screen, seek_speeds=None):
    """Follow the center line until a JUNCTION or LEAF is confirmed.

    Same debounce + lost-line recovery rules as the full solver, but with no
    object detection so the loop stays simple for tuning. Live raw/bits are
    pushed to the LCD on a throttle so PIL updates don't disturb timing.

    seek_speeds: (left, right) wheel speeds set right after a turn. The turn
        DECISION is committed: dead-end (LEAF) detection is OFF and the robot
        keeps PIVOTING in the decided direction (these speeds) until the CENTER
        sensor catches the new line. White during this phase means "still
        turning", never a dead end (no bogus U-turn). It does NOT drive straight
        to hunt for the line. Only once the center sensor reacquires the line
        does line tracing + dead-end detection resume.
    """
    if seek_speeds is not None:
        # 회전 결정 유지: 결정한 방향으로 계속 돌며 중앙 센서가 선을 잡을 때까지 기다린다.
        screen.event("hold turn, seek line")
        start = time.time()
        deadline = start + config.UTURN_TIMEOUT_SECONDS   # 안전 상한(무한 회전 방지)
        last_disp = 0.0
        hw.drive(seek_speeds[0], seek_speeds[1], apply_trim=False)
        while True:
            if hw.abort_requested():
                raise Aborted()
            raw = hw.read_reflect()
            bits = bits_from_raw(raw, config.line_thresholds())
            now = time.time()
            if now - last_disp >= 0.2:
                screen.status_line("seek raw{} bits{}".format(tuple(raw), bits))
                last_disp = now
            if bits[1]:                       # 중앙 센서가 선 포착 → 회전 끝, 트레이싱 시작
                hw.stop()
                if config.POST_TURN_SENSOR_SETTLE_SECONDS:
                    time.sleep(config.POST_TURN_SENSOR_SETTLE_SECONDS)
                screen.event("line found -> trace")
                break
            if now >= deadline:
                hw.stop()
                raise RuntimeError("회전 유지 중 라인을 다시 잡지 못했습니다(타임아웃)")
            time.sleep(config.LOOP_DELAY)
    else:
        # Brief nudge off the junction we just turned on (avoid re-confirming it).
        hw.drive(config.FOLLOW_SPEED, config.FOLLOW_SPEED)
        time.sleep(config.CLEAR_JUNCTION_SECONDS + config.POST_MOVE_SENSOR_SETTLE_SECONDS)

    deb = ArrivalDebouncer(config.JUNCTION_CONFIRM_SAMPLES,
                           config.LEAF_CONFIRM_SAMPLES,
                           config.EVENT_DEBOUNCE_MODE)
    leaf_since = None
    last_disp = 0.0

    while True:
        if hw.abort_requested():
            raise Aborted()

        raw = hw.read_reflect()
        bits = bits_from_raw(raw, config.line_thresholds())
        ev = event_kind(bits)
        confirmed = deb.push(bits)

        now = time.time()
        if now - last_disp >= 0.2:
            screen.status_line("raw{} bits{}".format(tuple(raw), bits))
            last_disp = now

        if ev == LEAF:
            # Might just be a brief line loss: creep slowly to re-acquire, then
            # stop and re-sample in place (don't ram a real dead-end wall).
            if leaf_since is None:
                leaf_since = time.time()
            recovered = (time.time() - leaf_since) >= config.LOST_LINE_RECOVERY_SECONDS
            if confirmed == LEAF and recovered:
                hw.stop()
                time.sleep(config.POST_STOP_SETTLE_SECONDS)
                return LEAF, bits
            if recovered:
                hw.stop()
            else:
                hw.drive(config.LOST_LINE_RECOVERY_SPEED, config.LOST_LINE_RECOVERY_SPEED)
        else:
            leaf_since = None
            if confirmed == JUNCTION:
                hw.stop()
                time.sleep(config.POST_STOP_SETTLE_SECONDS)
                return JUNCTION, bits
            if ev == JUNCTION:
                # Creep until the pattern is confirmed (settle on junction center).
                hw.drive(config.FOLLOW_SPEED, config.FOLLOW_SPEED)
            else:
                lspeed, rspeed = follow_speeds(raw, bits)
                hw.drive(lspeed, rspeed)

        time.sleep(config.LOOP_DELAY)


# =============================================================================
# Main
# =============================================================================
def wait_for_start(hw, screen):
    if not config.WAIT_FOR_BUTTON:
        return
    screen.splash("LINE TRACE TEST", "", "START on YELLOW", "press ENTER", "back = abort")
    while True:
        if hw.abort_requested():
            raise Aborted()
        if hw.enter_pressed():
            time.sleep(0.35)
            return
        time.sleep(0.05)


def run():
    try:
        config.validate_config()
    except ValueError as exc:
        print(exc, file=sys.stderr)
        return 2

    screen = Screen()
    hw = Ev3Hardware()
    # Reuse the tuned turn / color-read primitives, but keep them quiet (we do
    # our own ASCII status on the LCD).
    motion = Ev3Motion(hw, verbose=False)

    try:
        wait_for_start(hw, screen)
        hw.beep_start()
        screen.event("START on YELLOW")
        step = 0
        next_seek = None   # 직전 회전 방향(회전 직후엔 그 방향으로 회전 유지하며 선을 찾음)
        while True:
            step += 1
            kind, bits = follow_to_node(hw, screen, seek_speeds=next_seek)
            next_seek = None

            if kind == LEAF:
                screen.event("s{} DEAD END, read..".format(step))
                color = motion.read_node_color()

                if color == config.GOAL_COLOR:
                    screen.event("s{} RED=GOAL -> DONE".format(step))
                    hw.beep_ok()
                    break

                if color == config.CHECKPOINT_COLOR:
                    screen.event("s{} BLUE=CHK buzz+U".format(step))
                    buzz_up(hw)
                else:
                    screen.event("s{} c{} node -> U".format(step, color))
                motion.turn(UTURN)
                next_seek = seek_speeds_for(UTURN)

            else:  # JUNCTION
                token = choose_turn(bits)
                screen.event("s{} JCT {} -> {}".format(step, bits, TOKEN_NAME[token]))
                motion.pre_turn_nudge(token)
                motion.turn(token)
                next_seek = seek_speeds_for(token)

        hw.stop()
        return 0

    except Aborted:
        hw.stop()
        screen.event("ABORTED - stopped")
        print("aborted")
        return 130
    except KeyboardInterrupt:
        hw.stop()
        screen.event("ABORTED - stopped")
        print("aborted")
        return 130
    except Exception as exc:
        hw.stop()
        screen.event("ERR " + str(exc)[:22])
        print("error: {}".format(exc), file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(run())
