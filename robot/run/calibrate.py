#!/usr/bin/env python3
"""On-brick calibrator for the EV3 maze robot.

All LCD-facing text in this file is ASCII because the EV3 display font in this
setup cannot render Korean reliably.
"""

from __future__ import print_function

import argparse
import importlib
import os
import re
import shutil
import sys
import time
from datetime import datetime

import config
import solver
from solver import JUNCTION, LEAF


POLL_SECONDS = 0.05
LIVE_SECONDS = 0.10
ACTION_RESULT_SECONDS = 1.2

# The EV3 brick's physical Back button (top-left) is grabbed by brickman and
# kills the running program, so we never rely on it for navigation. Instead the
# "back / exit" action is a two-button chord: hold LEFT + RIGHT together for
# CHORD_HOLD_SECONDS. The hold requirement keeps an accidental simultaneous tap
# (common while nudging values with LEFT/RIGHT) from escaping the screen.
CHORD_BUTTONS = ("left", "right")
CHORD_HOLD_SECONDS = 0.4
BACK_HINT = "hold L+R=back"
EXIT_HINT = "hold L+R=exit"
DEFAULT_STRAIGHT_SECONDS = 2.0
DEBUG_LINE_MAX_SECONDS = getattr(config, "DEBUG_LINE_MAX_SECONDS", 20.0)
DEBUG_111_TIMEOUT_SECONDS = 10.0
DEBUG_LEFT_MAX_STEPS = getattr(config, "DEBUG_LEFT_MAX_STEPS", 80)
DEBUG_LEFT_MAX_SECONDS = getattr(config, "DEBUG_LEFT_MAX_SECONDS", 180.0)

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_PATH = os.path.join(BASE_DIR, "config.py")
CONFIG_BAK_PATH = os.path.join(BASE_DIR, "config.py.bak")

COLOR_NAMES = {
    0: "NONE",
    1: "BLACK",
    2: "BLUE",
    3: "GREEN",
    4: "YELLOW",
    5: "RED",
    6: "WHITE",
    7: "BROWN",
}

TOKEN_ITEMS = [
    ("L", solver.LEFT),
    ("R", solver.RIGHT),
    ("U", solver.UTURN),
    ("S", solver.STRAIGHT),
]

MEASURE_ITEMS = [
    ("SENSOR_RAW", "Sensor raw"),
    ("SENSOR_BITS", "Sensor bits"),
    ("CENTER_COLOR", "Center color"),
    ("ULTRASONIC", "Ultrasonic"),
    ("BUTTON_STATE", "Buttons"),
]

ACTION_ITEMS = [
    ("FOLLOW_ONCE", "Follow to node"),
    ("STRAIGHT_TRIM_ON", "Straight trim ON"),
    ("STRAIGHT_TRIM_OFF", "Straight trim OFF"),
    ("TURN_LEFT", "Left turn 90"),
    ("TURN_RIGHT", "Right turn 90"),
    ("TURN_UTURN", "U-turn 180"),
    ("TURN_STRAIGHT", "Straight nudge"),
    ("SENSE_EXITS", "Sense exits"),
    ("READ_COLOR", "Read node color"),
    ("GRIP_CLOSE_LIFT", "Grip close+lift"),
    ("GRIP_RELEASE", "Grip release"),
    ("DETECT_AND_GRIP", "Detect+grip"),
]

DEBUG_ITEMS = [
    ("LINE_ONLY", "Line only"),
    ("NEXT_NODE", "Next node only"),
    ("JUNCTION_111", "111 only"),
    ("PATTERN_STREAM", "Pattern stream"),
    ("PEEK_REPEAT", "Peek repeat"),
    ("LEFT_RULE_RUN", "Left rule run"),
    ("TURN_REPEAT", "Turn repeat"),
    ("GRIP_JOG", "Grip jog"),
    ("GRIP_ONLY", "Grip only"),
    ("RELEASE_ONLY", "Release only"),
    ("DIST_STREAM", "Distance stream"),
    ("COLOR_REPEAT", "Color repeat"),
]

SYSTEM_ITEMS = [
    ("VALIDATE", "Validate config"),
    ("VIEW_CONFIG", "View core config"),
    ("LOG_PATH", "Log path"),
    ("BEEP_TEST", "Beep test"),
    ("CONFIG_FINAL", "Log final config"),
    ("EXIT", "Exit"),
]

ADJUST_GROUPS = [
    ("SENSOR", [
        ("LINE_THRESHOLD", 1),
        ("LEFT_LINE_THRESHOLD", 1),
        ("CENTER_LINE_THRESHOLD", 1),
        ("RIGHT_LINE_THRESHOLD", 1),
    ]),
    ("FOLLOW", [
        ("FOLLOW_SPEED", 1),
        ("KP", 0.05),
        ("SIDE_CORRECTION", 1),
    ]),
    ("TRIM", [
        ("LEFT_MOTOR_TRIM", 0.01),
        ("RIGHT_MOTOR_TRIM", 0.01),
    ]),
    ("LEFT_TURN", [
        ("LEFT_TURN_SPEED", 1),
        ("LEFT_TURN_IGNORE_SECONDS", 0.01),
        ("LEFT_TURN_MIN_SECONDS", 0.01),
        ("LEFT_TURN_TIMEOUT_SECONDS", 0.05),
        ("LEFT_TURN_REQUIRE_LINE_CLEAR", 1),
    ]),
    ("RIGHT_TURN", [
        ("RIGHT_TURN_SPEED", 1),
        ("RIGHT_TURN_IGNORE_SECONDS", 0.01),
        ("RIGHT_TURN_MIN_SECONDS", 0.01),
        ("RIGHT_TURN_TIMEOUT_SECONDS", 0.05),
        ("RIGHT_TURN_REQUIRE_LINE_CLEAR", 1),
    ]),
    ("UTURN", [
        ("UTURN_SPEED", 1),
        ("UTURN_IGNORE_SECONDS", 0.01),
        ("UTURN_MIN_SECONDS", 0.01),
        ("UTURN_TIMEOUT_SECONDS", 0.05),
        ("UTURN_REQUIRE_LINE_CLEAR", 1),
    ]),
    ("JUNCTION", [
        ("JUNCTION_CONFIRM_SAMPLES", 1),
        ("JUNCTION_CENTERING_SECONDS", 0.01),
        ("STRAIGHT_NUDGE_SECONDS", 0.01),
        ("CLEAR_JUNCTION_SECONDS", 0.01),
        ("PEEK_FORWARD_SECONDS", 0.01),
        ("PEEK_BACK_SECONDS", 0.01),
        ("PEEK_SETTLE_SECONDS", 0.01),
        ("PEEK_SAMPLES", 1),
        ("PEEK_SENSOR_SETTLE_SECONDS", 0.01),
    ]),
    ("LEAF", [
        ("LEAF_CONFIRM_SAMPLES", 1),
        ("LOST_LINE_RECOVERY_SECONDS", 0.01),
        ("LOST_LINE_RECOVERY_SPEED", 1),
    ]),
    ("COLOR", [
        ("START_COLOR", 1),
        ("CHECKPOINT_COLOR", 1),
        ("GOAL_COLOR", 1),
        ("COLOR_CONFIRM_SAMPLES", 1),
        ("COLOR_MODE_SETTLE_SECONDS", 0.01),
        ("COLOR_DUMMY_READS", 1),
        ("COLOR_MODE_RESTORE_SETTLE_SECONDS", 0.01),
    ]),
    ("OBJECT_GRIP", [
        ("OBSTACLE_DISTANCE_CM", 0.5),
        ("OBSTACLE_MIN_VALID_CM", 0.5),
        ("OBSTACLE_CONFIRM_SAMPLES", 1),
        ("GRIP_CLOSE_DEGREES", 5),
        ("GRIP_SPEED", 1),
        ("LIFT_DEGREES", 5),
        ("POST_GRIP_SETTLE_SECONDS", 0.01),
        ("POST_RELEASE_SETTLE_SECONDS", 0.01),
    ]),
]

ADJUST_ITEMS = []
for _group, _items in ADJUST_GROUPS:
    for _name, _step in _items:
        ADJUST_ITEMS.append((_group, _name, _step))

# Default adjust step per config name (reused by the ACTION live-tuning screens).
STEP_BY_NAME = {name: step for _grp, name, step in ADJUST_ITEMS}

# For each ACTION, the config values that actually shape its behaviour. The
# action screen shows these and lets the operator tweak them (UP/DOWN) and
# re-run (ENTER) without leaving the screen, so a turn/grip/follow can be dialed
# in iteratively. Order = most-impactful first.
ACTION_PARAMS = {
    "FOLLOW_ONCE": ["FOLLOW_SPEED", "KP", "SIDE_CORRECTION", "LINE_THRESHOLD"],
    "STRAIGHT_TRIM_ON": ["FOLLOW_SPEED", "LEFT_MOTOR_TRIM", "RIGHT_MOTOR_TRIM"],
    "STRAIGHT_TRIM_OFF": ["FOLLOW_SPEED"],
    "TURN_LEFT": ["LEFT_TURN_SPEED", "LEFT_TURN_IGNORE_SECONDS",
                  "LEFT_TURN_MIN_SECONDS", "LEFT_TURN_TIMEOUT_SECONDS",
                  "LEFT_TURN_REQUIRE_LINE_CLEAR"],
    "TURN_RIGHT": ["RIGHT_TURN_SPEED", "RIGHT_TURN_IGNORE_SECONDS",
                   "RIGHT_TURN_MIN_SECONDS", "RIGHT_TURN_TIMEOUT_SECONDS",
                   "RIGHT_TURN_REQUIRE_LINE_CLEAR"],
    "TURN_UTURN": ["UTURN_SPEED", "UTURN_IGNORE_SECONDS", "UTURN_MIN_SECONDS",
                   "UTURN_TIMEOUT_SECONDS", "UTURN_REQUIRE_LINE_CLEAR"],
    "TURN_STRAIGHT": ["FOLLOW_SPEED", "STRAIGHT_NUDGE_SECONDS"],
    "SENSE_EXITS": ["PEEK_FORWARD_SECONDS", "PEEK_BACK_SECONDS",
                    "PEEK_SETTLE_SECONDS", "PEEK_SAMPLES",
                    "PEEK_SENSOR_SETTLE_SECONDS"],
    "READ_COLOR": ["COLOR_MODE_SETTLE_SECONDS", "COLOR_DUMMY_READS",
                   "COLOR_MODE_RESTORE_SETTLE_SECONDS", "COLOR_CONFIRM_SAMPLES"],
    "GRIP_CLOSE_LIFT": ["GRIP_CLOSE_DEGREES", "GRIP_SPEED", "LIFT_DEGREES",
                        "POST_GRIP_SETTLE_SECONDS"],
    "GRIP_RELEASE": ["GRIP_CLOSE_DEGREES", "LIFT_DEGREES", "GRIP_SPEED",
                     "POST_RELEASE_SETTLE_SECONDS"],
    "DETECT_AND_GRIP": ["OBSTACLE_DISTANCE_CM", "OBSTACLE_MIN_VALID_CM",
                        "GRIP_CLOSE_DEGREES", "GRIP_SPEED", "LIFT_DEGREES"],
}


def default_step(value):
    if isinstance(value, bool):
        return 1
    if isinstance(value, int):
        return 1
    return 0.01


def action_params(step):
    """[(config_name, adjust_step), ...] for the given ACTION step."""
    out = []
    for name in ACTION_PARAMS.get(step, []):
        out.append((name, STEP_BY_NAME.get(name, default_step(getattr(config, name)))))
    return out

MENU_GROUPS = [
    ("MEASURE", MEASURE_ITEMS),
    ("ACTION", ACTION_ITEMS),
    ("DEBUG", DEBUG_ITEMS),
    ("ADJUST", [(name, name) for _grp, name, _step in ADJUST_ITEMS]),
    ("SYSTEM", SYSTEM_ITEMS),
]


class UserAbort(Exception):
    pass


def now_iso():
    return datetime.now().replace(microsecond=0).isoformat()


def clamp(value, low, high):
    return max(low, min(high, value))


def format_value(value):
    if isinstance(value, bool):
        return "true" if value else "false"
    if value is None:
        return "None"
    if isinstance(value, (tuple, list)):
        return "(" + ",".join(format_value(v) for v in value) + ")"
    text = str(value)
    return text.replace(" ", "_").replace("\t", "_")


def config_literal(value):
    if isinstance(value, bool):
        return "True" if value else "False"
    if value is None:
        return "None"
    if isinstance(value, str):
        return repr(value)
    return repr(value)


class Screen(object):
    def __init__(self):
        self.display = None
        self.last_text = None
        try:
            from ev3dev2.display import Display
            self.display = Display()
        except Exception:
            self.display = None

    def show(self, lines, footer=BACK_HINT):
        # Legacy "BACK=..." hint lines referred to the physical Back button,
        # which now kills the program; drop them and show the chord footer at
        # the bottom of every screen instead.
        content = [str(line)[:28] for line in lines if not str(line).startswith("BACK=")]
        content = content[:7]
        if footer is not None:
            content.append(str(footer)[:28])
        clean = content[:8]
        text = "\n".join(clean)
        if text != self.last_text:
            print("[LCD]\n" + text)
            self.last_text = text
        if self.display is None:
            return
        try:
            self.display.clear()
            y = 0
            for line in clean:
                self.display.text_pixels(line, x=0, y=y, clear_screen=False)
                y += 14
            self.display.update()
        except Exception:
            self.display = None


class ButtonEdges(object):
    """Edge-detecting button reader.

    Single buttons (up/down/left/right/enter) fire once on release. The "back"
    event is the LEFT+RIGHT chord held for CHORD_HOLD_SECONDS — see the comment
    near CHORD_BUTTONS for why the physical Back button is not used. While the
    chord buttons are engaged, their individual edges are suppressed so holding
    the chord never also registers a stray left/right.
    """

    def __init__(self, hw):
        self.hw = hw
        self.prev = self.hw.button_state()
        self._chord_since = None
        self._chord_fired = False
        self._suppress = False

    def state(self):
        return self.hw.button_state()

    def poll(self):
        cur = self.hw.button_state()
        a, b = CHORD_BUTTONS
        chord_now = cur.get(a) and cur.get(b)
        edge = None

        if chord_now:
            # Both chord buttons down: don't emit singles; arm/measure the hold.
            self._suppress = True
            if self._chord_since is None:
                self._chord_since = time.time()
            if not self._chord_fired and time.time() - self._chord_since >= CHORD_HOLD_SECONDS:
                self._chord_fired = True
                edge = "back"
            self.prev = cur
            return edge

        # Chord not (fully) held right now.
        self._chord_since = None
        self._chord_fired = False
        suppress = self._suppress
        if not any(cur.values()):
            self._suppress = False  # cleared for next poll; this one stays suppressed
        if not suppress:
            for name in ("up", "down", "left", "right", "enter"):
                if self.prev.get(name) and not cur.get(name):
                    edge = name
                    break
        # If the physical Back button ever does reach us (e.g. launched over SSH
        # rather than from brickman), honour it as back too.
        if cur.get("back"):
            edge = "back"
        self.prev = cur
        return edge

    def wait(self):
        while True:
            edge = self.poll()
            if edge:
                return edge
            time.sleep(POLL_SECONDS)


class TuneLog(object):
    def __init__(self):
        log_dir = os.path.join(BASE_DIR, "logs")
        if not os.path.isdir(log_dir):
            os.makedirs(log_dir)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = os.path.join(log_dir, "tune_{}.log".format(stamp))
        self.fp = open(self.path, "a", encoding="utf-8")

    def write(self, group, step, run_id, seq, fields=None, note="ok"):
        fields = fields or {}
        pairs = ["run={}".format(run_id), "seq={}".format(seq)]
        for key in sorted(fields):
            pairs.append("{}={}".format(key, format_value(fields[key])))
        line = "{}\t{}\t{}\t{}\t{}\n".format(
            now_iso(), group, step, " ".join(pairs), note)
        self.fp.write(line)
        self.fp.flush()
        print(line.rstrip())

    def write_config_final(self):
        fields = {}
        for name in sorted(dir(config)):
            if not name.isupper():
                continue
            value = getattr(config, name)
            if isinstance(value, (bool, int, float, str, tuple, list)) or value is None:
                fields[name] = value
        self.write("SYSTEM", "CONFIG_FINAL", 0, 0, fields, "final")

    def close(self):
        self.fp.close()


class Calibrator(object):
    def __init__(self, hw, write_enabled=False):
        self.hw = hw
        self.motion = solver.Ev3Motion(hw)
        self.write_enabled = write_enabled
        self.screen = Screen()
        self.buttons = ButtonEdges(hw)
        self.log = TuneLog()
        self.group_index = 0
        self.item_indices = [0 for _ in MENU_GROUPS]
        self.run_id = 0
        self.exiting = False

    def run(self):
        self.hw.beep_ok()
        try:
            while not self.exiting:
                self.render_menu()
                key = self.buttons.wait()
                if key == "left":
                    self.group_index = (self.group_index - 1) % len(MENU_GROUPS)
                elif key == "right":
                    self.group_index = (self.group_index + 1) % len(MENU_GROUPS)
                elif key == "up":
                    self.move_item(-1)
                elif key == "down":
                    self.move_item(1)
                elif key == "enter":
                    self.run_selected()
                elif key == "back":
                    if self.confirm_exit():
                        self.exiting = True
        finally:
            try:
                self.hw.stop_all()
            finally:
                self.log.write_config_final()
                self.hw.beep()
                self.log.close()

    def current_group(self):
        return MENU_GROUPS[self.group_index]

    def current_item(self):
        group, items = self.current_group()
        return items[self.item_indices[self.group_index]]

    def move_item(self, delta):
        _group, items = self.current_group()
        self.item_indices[self.group_index] = (
            self.item_indices[self.group_index] + delta) % len(items)

    def render_menu(self):
        group, items = self.current_group()
        index = self.item_indices[self.group_index]
        lines = ["{} {}/{}".format(group, index + 1, len(items))]
        for offset in (-2, -1, 0, 1, 2):
            pos = index + offset
            if 0 <= pos < len(items):
                mark = ">" if pos == index else " "
                lines.append("{} {}".format(mark, items[pos][1]))
        lines.append("LR=grp UD=item")
        lines.append("ENTER=select")
        self.screen.show(lines, footer=EXIT_HINT)

    def confirm_exit(self):
        self.screen.show(["EXIT CALIBRATOR?", "", "ENTER = yes",
                          "any other = no"], footer=None)
        while True:
            key = self.buttons.wait()
            if key == "enter":
                return True
            if key in ("up", "down", "left", "right", "back"):
                return False

    def run_selected(self):
        group, _items = self.current_group()
        step, _label = self.current_item()
        self.run_id += 1
        self.hw.beep()
        try:
            self.hw.stop_all()
            if group == "MEASURE":
                self.run_measure(step, self.run_id)
            elif group == "ACTION":
                self.run_action_screen(step, self.run_id)
            elif group == "DEBUG":
                self.run_debug(step, self.run_id)
            elif group == "ADJUST":
                self.run_adjust(self.item_indices[self.group_index], self.run_id)
            elif group == "SYSTEM":
                self.run_system(step, self.run_id)
        except (UserAbort, solver.Aborted):
            self.hw.stop_all()
            self.hw.beep_abort()
            self.log.write(group, step, self.run_id, 0, {}, "aborted")
            self.show_pause(["ABORTED", "BACK=menu"], 0.8)
        except Exception as exc:
            self.hw.stop_all()
            self.hw.beep_error()
            self.log.write(group, step, self.run_id, 0, {"err": repr(exc)}, "error")
            self.show_pause(["ERROR", str(exc)[:26], "BACK=menu"], 1.5)
        finally:
            self.hw.stop_all()

    def show_pause(self, lines, seconds):
        self.screen.show(lines)
        end = time.time() + seconds
        while time.time() < end:
            if self.buttons.poll() == "back":
                return
            time.sleep(POLL_SECONDS)

    def check_abort_edge(self):
        key = self.buttons.poll()
        if key == "back":
            self.hw.stop_all()
            self.hw.beep_abort()
            raise UserAbort()
        return key

    # ------------------------------------------------------------------
    # MEASURE
    # ------------------------------------------------------------------
    def run_measure(self, step, run_id):
        if step == "SENSOR_RAW":
            self.measure_sensor_raw(run_id)
        elif step == "SENSOR_BITS":
            self.measure_sensor_bits(run_id)
        elif step == "CENTER_COLOR":
            self.measure_center_color(run_id)
        elif step == "ULTRASONIC":
            self.measure_ultrasonic(run_id)
        elif step == "BUTTON_STATE":
            self.measure_buttons(run_id)

    def measure_sensor_raw(self, run_id):
        tags = ["white", "black", "line", "free"]
        tag_i = 0
        seq = 0
        seen = {}
        while True:
            raw = self.hw.read_reflect()
            if "white" in seen and "black" in seen:
                cand = tuple(int((w + b) / 2) for w, b in zip(seen["white"], seen["black"]))
                cand_line = "thr {} {} {}".format(cand[0], cand[1], cand[2])
            else:
                cand_line = "thr need white+black"
            self.screen.show([
                "SENSOR RAW",
                "L C R = {} {} {}".format(raw[0], raw[1], raw[2]),
                "tag={}".format(tags[tag_i]),
                cand_line,
                "UD=tag ENTER=snap",
                "BACK=menu",
            ])
            key = self.check_abort_edge()
            if key == "up":
                tag_i = (tag_i - 1) % len(tags)
            elif key == "down":
                tag_i = (tag_i + 1) % len(tags)
            elif key == "enter":
                seq += 1
                tag = tags[tag_i]
                seen[tag] = raw
                self.log.write("MEASURE", "SENSOR_RAW", run_id, seq,
                               {"L": raw[0], "C": raw[1], "R": raw[2], "tag": tag},
                               "snapshot")
                self.hw.beep_snapshot()
            time.sleep(LIVE_SECONDS)

    def measure_sensor_bits(self, run_id):
        seq = 0
        while True:
            raw = self.hw.read_reflect()
            bits = solver.bits_from_raw(raw, config.line_thresholds())
            self.screen.show([
                "SENSOR BITS",
                "raw {} {} {}".format(raw[0], raw[1], raw[2]),
                "bits {}{}{}".format(bits[0], bits[1], bits[2]),
                "thr {}".format(config.line_thresholds()),
                "ENTER=snap",
                "BACK=menu",
            ])
            key = self.check_abort_edge()
            if key == "enter":
                seq += 1
                self.log.write("MEASURE", "SENSOR_BITS", run_id, seq,
                               {"raw": raw, "bits": bits}, "snapshot")
                self.hw.beep_snapshot()
            time.sleep(LIVE_SECONDS)

    def measure_center_color(self, run_id):
        tags = ["start", "check", "goal", "free"]
        tag_i = 0
        seq = 0
        while True:
            color = self.hw.read_center_color()
            self.screen.show([
                "CENTER COLOR",
                "color={} {}".format(color, COLOR_NAMES.get(color, "?")),
                "tag={}".format(tags[tag_i]),
                "UD=tag ENTER=snap",
                "BACK=menu",
            ])
            key = self.check_abort_edge()
            if key == "up":
                tag_i = (tag_i - 1) % len(tags)
            elif key == "down":
                tag_i = (tag_i + 1) % len(tags)
            elif key == "enter":
                seq += 1
                self.log.write("MEASURE", "CENTER_COLOR", run_id, seq,
                               {"color": color, "tag": tags[tag_i]}, "snapshot")
                self.hw.beep_snapshot()
            time.sleep(LIVE_SECONDS)

    def measure_ultrasonic(self, run_id):
        seq = 0
        while True:
            dist = self.hw.distance_cm()
            detect = config.OBSTACLE_MIN_VALID_CM <= dist < config.OBSTACLE_DISTANCE_CM
            self.screen.show([
                "ULTRASONIC",
                "cm={:.1f}".format(dist),
                "detect={}".format("YES" if detect else "NO"),
                "range {:.1f}..{:.1f}".format(
                    config.OBSTACLE_MIN_VALID_CM, config.OBSTACLE_DISTANCE_CM),
                "ENTER=snap",
                "BACK=menu",
            ])
            key = self.check_abort_edge()
            if key == "enter":
                seq += 1
                self.log.write("MEASURE", "ULTRASONIC", run_id, seq,
                               {"cm": round(dist, 2), "detect": detect}, "snapshot")
                self.hw.beep_snapshot()
            time.sleep(LIVE_SECONDS)

    def measure_buttons(self, run_id):
        seq = 0
        while True:
            state = self.buttons.state()
            pressed = [k for k in ("up", "down", "left", "right", "enter", "back") if state[k]]
            self.screen.show([
                "BUTTON STATE",
                "pressed:",
                ",".join(pressed) if pressed else "none",
                "ENTER=snap",
                "BACK=menu",
            ])
            key = self.check_abort_edge()
            if key == "enter":
                seq += 1
                self.log.write("MEASURE", "BUTTON_STATE", run_id, seq, state, "snapshot")
                self.hw.beep_snapshot()
            time.sleep(LIVE_SECONDS)

    # ------------------------------------------------------------------
    # ACTION
    # ------------------------------------------------------------------
    def run_action_screen(self, step, run_id):
        """Run an ACTION while showing/editing the config values it depends on.

        UP/DOWN nudge the selected config value (live, in RAM), LEFT/RIGHT pick
        which value, ENTER runs the action with the current settings. So you can
        e.g. bump LEFT_TURN_MIN_SECONDS and re-run the turn until 90 is clean,
        all without leaving the screen. Edits stay in memory for the rest of the
        session (the run log records the values each run used); use the ADJUST
        menu with --write to persist a value to config.py.
        """
        params = action_params(step)
        originals = {name: getattr(config, name) for name, _ in params}
        cursor = 0
        seq = 0
        while True:
            self.render_action_screen(step, params, originals, cursor, seq)
            key = self.buttons.wait()
            if key == "back":
                return
            if not params:
                if key == "enter":
                    seq += 1
                    self.execute_action(step, run_id, seq)
                continue
            if key == "left":
                cursor = (cursor - 1) % len(params)
            elif key == "right":
                cursor = (cursor + 1) % len(params)
            elif key in ("up", "down"):
                name, pstep = params[cursor]
                cur_value = getattr(config, name)
                new_value = self.adjust_value(cur_value, originals[name], pstep,
                                              1 if key == "up" else -1)
                self.set_config_live(name, new_value)
            elif key == "enter":
                seq += 1
                self.execute_action(step, run_id, seq)

    def render_action_screen(self, step, params, originals, cursor, seq):
        lines = ["ACT {}".format(step)[:28]]
        if not params:
            lines += ["uses config.py", "(no tunable params)", "ENTER=run"]
            self.screen.show(lines)
            return
        name, pstep = params[cursor]
        cur_value = getattr(config, name)
        old_value = originals[name]
        changed = "*" if cur_value != old_value else "param"
        lines.append("{} {}/{}  run={}".format(changed, cursor + 1, len(params), seq))
        lines.append(name)  # full name on its own line
        lines.append("now={}".format(format_value(cur_value)))
        lines.append("was={} step={}".format(format_value(old_value), pstep))
        lines.append("UD=value LR=param ENT=run")
        self.screen.show(lines)

    def set_config_live(self, name, value):
        """Apply a config change in memory only, rejecting values that fail
        validate_config(). Returns True on success."""
        old = getattr(config, name)
        if value == old:
            return True
        setattr(config, name, value)
        try:
            config.validate_config()
            return True
        except Exception as exc:
            setattr(config, name, old)
            self.hw.beep_error()
            self.show_pause(["INVALID VALUE", str(exc)[:26]], 1.0)
            return False

    def execute_action(self, step, run_id, seq):
        self.hw.beep_start()
        start = time.time()
        try:
            fields = self.run_action_once(step)
        except (UserAbort, solver.Aborted):
            self.hw.stop_all()
            self.log.write("ACTION", step, run_id, seq,
                           self.with_params(step, {"elapsed": round(time.time() - start, 3)}),
                           "aborted")
            self.hw.beep_abort()
            self.show_pause(["ABORTED", step], ACTION_RESULT_SECONDS)
            return
        except Exception as exc:
            self.hw.stop_all()
            self.log.write("ACTION", step, run_id, seq,
                           self.with_params(step, {"err": repr(exc),
                                                   "elapsed": round(time.time() - start, 3)}),
                           "error")
            self.hw.beep_error()
            self.show_pause(["ERROR", step, str(exc)[:26]], ACTION_RESULT_SECONDS)
            return
        fields["elapsed"] = round(time.time() - start, 3)
        self.log.write("ACTION", step, run_id, seq, self.with_params(step, fields), "ok")
        self.hw.beep_ok()
        self.show_pause(["DONE {}".format(step),
                         "elapsed={:.2f}".format(fields["elapsed"])]
                        + self.action_result_lines(step, fields),
                        ACTION_RESULT_SECONDS)

    def with_params(self, step, fields):
        """Stamp the config values this run used onto the log fields, so each
        result row ties back to the settings that produced it."""
        out = dict(fields)
        for name, _ in action_params(step):
            out[name] = getattr(config, name)
        return out

    def action_result_lines(self, step, fields):
        skip = set(["elapsed"]) | set(name for name, _ in action_params(step))
        out = []
        for key in sorted(fields):
            if key in skip:
                continue
            out.append("{}={}".format(key, format_value(fields[key]))[:28])
            if len(out) >= 3:
                break
        return out

    def run_action_once(self, step):
        if step == "FOLLOW_ONCE":
            arr = self.motion.follow_to_node("CAL")
            raw = self.hw.read_reflect()
            bits = solver.bits_from_raw(raw, config.line_thresholds())
            return {"kind": arr.kind, "raw": raw, "bits": bits}
        if step == "STRAIGHT_TRIM_ON":
            return self.drive_straight(DEFAULT_STRAIGHT_SECONDS, True)
        if step == "STRAIGHT_TRIM_OFF":
            return self.drive_straight(DEFAULT_STRAIGHT_SECONDS, False)
        if step == "TURN_LEFT":
            self.motion.turn(solver.LEFT)
            return {"token": "L"}
        if step == "TURN_RIGHT":
            self.motion.turn(solver.RIGHT)
            return {"token": "R"}
        if step == "TURN_UTURN":
            self.motion.turn(solver.UTURN)
            return {"token": "U"}
        if step == "TURN_STRAIGHT":
            self.motion.turn(solver.STRAIGHT)
            return {"token": "S"}
        if step == "SENSE_EXITS":
            exits, cross = self.motion.sense_exits()
            return {"L": exits["L"], "S": exits["S"], "R": exits["R"], "cross": cross}
        if step == "READ_COLOR":
            color = self.motion.read_node_color()
            return {"color": color, "name": COLOR_NAMES.get(color, "?")}
        if step == "GRIP_CLOSE_LIFT":
            result = self.safe_grip_close_lift()
            return result
        if step == "GRIP_RELEASE":
            result = self.safe_grip_release()
            return result
        if step == "DETECT_AND_GRIP":
            dist = self.hw.distance_cm()
            detect = config.OBSTACLE_MIN_VALID_CM <= dist < config.OBSTACLE_DISTANCE_CM
            if not detect:
                return {"cm": round(dist, 2), "detect": False}
            result = self.safe_grip_close_lift()
            result["cm"] = round(dist, 2)
            result["detect"] = True
            return result
        raise ValueError(step)

    def drive_straight(self, seconds, trim):
        start = time.time()
        while time.time() - start < seconds:
            key = self.buttons.poll()
            if key == "back":
                raise UserAbort()
            self.motion._drive(config.FOLLOW_SPEED, config.FOLLOW_SPEED, trim=trim)
            self.screen.show([
                "DRIVE STRAIGHT",
                "trim={}".format("ON" if trim else "OFF"),
                "t={:.1f}/{:.1f}".format(time.time() - start, seconds),
                "BACK=stop",
            ])
            time.sleep(POLL_SECONDS)
        self.motion._stop()
        return {"seconds": seconds, "trim": trim}

    # ------------------------------------------------------------------
    # DEBUG
    # ------------------------------------------------------------------
    def run_debug(self, step, run_id):
        if step == "LINE_ONLY":
            self.debug_line_only(run_id)
        elif step == "NEXT_NODE":
            self.debug_next_node(run_id)
        elif step == "JUNCTION_111":
            self.debug_111(run_id)
        elif step == "PATTERN_STREAM":
            self.debug_pattern_stream(run_id)
        elif step == "PEEK_REPEAT":
            self.debug_peek_repeat(run_id)
        elif step == "LEFT_RULE_RUN":
            self.debug_left_rule(run_id)
        elif step == "TURN_REPEAT":
            self.debug_turn_repeat(run_id)
        elif step == "GRIP_JOG":
            self.debug_grip_jog(run_id)
        elif step == "GRIP_ONLY":
            self.debug_grip_only(run_id)
        elif step == "RELEASE_ONLY":
            self.debug_release_only(run_id)
        elif step == "DIST_STREAM":
            self.debug_distance_stream(run_id)
        elif step == "COLOR_REPEAT":
            self.debug_color_repeat(run_id)

    def debug_line_only(self, run_id):
        seq = 0
        start = time.time()
        last_log = 0
        self.hw.beep_start()
        while time.time() - start < DEBUG_LINE_MAX_SECONDS:
            key = self.buttons.poll()
            if key == "back":
                raise UserAbort()
            raw, bits = self.motion._read()
            lspeed, rspeed = self.motion._follow_speeds(raw, bits)
            self.motion._drive(lspeed, rspeed)
            now = time.time()
            if now - last_log >= config.DEBUG_LOG_INTERVAL_SECONDS:
                seq += 1
                last_log = now
                self.log.write("DEBUG", "LINE_ONLY", run_id, seq,
                               {"raw": raw, "bits": bits,
                                "ls": round(lspeed, 2), "rs": round(rspeed, 2)},
                               "stream")
            self.screen.show([
                "LINE ONLY",
                "raw {} {} {}".format(raw[0], raw[1], raw[2]),
                "bits {}{}{}".format(bits[0], bits[1], bits[2]),
                "spd {:.0f} {:.0f}".format(lspeed, rspeed),
                "BACK=stop",
            ])
            time.sleep(config.LOOP_DELAY)
        self.motion._stop()
        self.log.write("DEBUG", "LINE_ONLY", run_id, seq + 1, {}, "limit")
        self.hw.beep_ok()

    def debug_next_node(self, run_id):
        self.confirm_debug_start("NEXT NODE")
        start = time.time()
        arr = self.motion.follow_to_node("DEBUG_NEXT")
        raw = self.hw.read_reflect()
        bits = solver.bits_from_raw(raw, config.line_thresholds())
        self.log.write("DEBUG", "NEXT_NODE", run_id, 1,
                       {"kind": arr.kind, "raw": raw, "bits": bits,
                        "elapsed": round(time.time() - start, 3)}, "ok")
        self.hw.beep_ok()
        self.show_pause(["NEXT NODE", "kind={}".format(arr.kind)], ACTION_RESULT_SECONDS)

    def debug_111(self, run_id):
        self.confirm_debug_start("111 ONLY")
        seq = 0
        count = 0
        start = time.time()
        while time.time() - start < DEBUG_111_TIMEOUT_SECONDS:
            key = self.buttons.poll()
            if key == "back":
                raise UserAbort()
            raw, bits = self.motion._read()
            if bits == (1, 1, 1):
                count += 1
            else:
                count = 0
            lspeed, rspeed = self.motion._follow_speeds(raw, bits)
            self.motion._drive(lspeed, rspeed)
            seq += 1
            self.screen.show([
                "111 ONLY",
                "bits {}{}{} count={}".format(bits[0], bits[1], bits[2], count),
                "need={}".format(config.JUNCTION_CONFIRM_SAMPLES),
                "BACK=stop",
            ])
            if count >= config.JUNCTION_CONFIRM_SAMPLES:
                self.motion._stop()
                self.log.write("DEBUG", "JUNCTION_111", run_id, seq,
                               {"raw": raw, "bits": bits}, "confirmed")
                self.hw.beep_ok()
                return
            if seq % 5 == 0:
                self.log.write("DEBUG", "JUNCTION_111", run_id, seq,
                               {"raw": raw, "bits": bits, "count": count}, "stream")
            time.sleep(config.LOOP_DELAY)
        self.motion._stop()
        self.hw.beep_error()
        self.log.write("DEBUG", "JUNCTION_111", run_id, seq,
                       {"count": count}, "timeout")

    def debug_pattern_stream(self, run_id):
        seq = 0
        deb = solver.ArrivalDebouncer(config.JUNCTION_CONFIRM_SAMPLES,
                                      config.LEAF_CONFIRM_SAMPLES,
                                      config.EVENT_DEBOUNCE_MODE)
        while True:
            raw, bits = self.motion._read()
            ev = solver.event_kind(bits)
            confirmed = deb.push(bits)
            self.screen.show([
                "PATTERN STREAM",
                "raw {} {} {}".format(raw[0], raw[1], raw[2]),
                "bits {}{}{}".format(bits[0], bits[1], bits[2]),
                "event={}".format(ev or "none"),
                "cnt={} ok={}".format(deb.count, confirmed or "none"),
                "BACK=menu",
            ])
            seq += 1
            self.log.write("DEBUG", "PATTERN_STREAM", run_id, seq,
                           {"raw": raw, "bits": bits, "event": ev or "none",
                            "count": deb.count, "confirmed": confirmed or "none"},
                           "stream")
            key = self.check_abort_edge()
            if key == "enter":
                self.hw.beep_snapshot()
            time.sleep(LIVE_SECONDS)

    def debug_peek_repeat(self, run_id):
        seq = 0
        while True:
            self.screen.show(["PEEK REPEAT", "ENTER=peek", "BACK=menu"])
            key = self.buttons.wait()
            if key == "back":
                return
            if key == "enter":
                seq += 1
                exits, cross = self.motion.sense_exits()
                self.log.write("DEBUG", "PEEK_REPEAT", run_id, seq,
                               {"L": exits["L"], "S": exits["S"],
                                "R": exits["R"], "cross": cross}, "ok")
                self.hw.beep_ok()

    def debug_left_rule(self, run_id):
        self.confirm_debug_start("LEFT RULE")
        seq = 0
        start = time.time()
        while seq < DEBUG_LEFT_MAX_STEPS and time.time() - start < DEBUG_LEFT_MAX_SECONDS:
            key = self.buttons.poll()
            if key == "back":
                raise UserAbort()
            seq += 1
            arr = self.motion.follow_to_node("DEBUG_LEFT")
            fields = {"kind": arr.kind}
            if arr.kind == LEAF:
                color = self.motion.read_node_color()
                fields["color"] = color
                if color == config.GOAL_COLOR:
                    self.log.write("DEBUG", "LEFT_RULE_RUN", run_id, seq, fields, "goal")
                    self.hw.beep_ok()
                    return
                self.motion.turn(solver.UTURN)
                fields["token"] = "U"
            else:
                exits, cross = self.motion.sense_exits()
                fields.update({"L": exits["L"], "S": exits["S"], "R": exits["R"],
                               "cross": cross})
                token = solver.UTURN
                token_name = "U"
                if exits["L"]:
                    token, token_name = solver.LEFT, "L"
                elif exits["S"]:
                    token, token_name = solver.STRAIGHT, "S"
                elif exits["R"]:
                    token, token_name = solver.RIGHT, "R"
                self.motion.turn(token)
                fields["token"] = token_name
            self.log.write("DEBUG", "LEFT_RULE_RUN", run_id, seq, fields, "step")
            self.screen.show(["LEFT RULE", "step={}".format(seq),
                              "last={}".format(fields.get("token", "?")),
                              "BACK=stop"])
        self.log.write("DEBUG", "LEFT_RULE_RUN", run_id, seq,
                       {"steps": seq}, "limit")
        self.hw.beep_error()

    def debug_turn_repeat(self, run_id):
        seq = 0
        token_i = 0
        while True:
            name, token = TOKEN_ITEMS[token_i]
            self.screen.show([
                "TURN REPEAT",
                "token={}".format(name),
                "LR=select",
                "ENTER=run",
                "BACK=menu",
            ])
            key = self.buttons.wait()
            if key == "back":
                return
            if key == "left":
                token_i = (token_i - 1) % len(TOKEN_ITEMS)
            elif key == "right":
                token_i = (token_i + 1) % len(TOKEN_ITEMS)
            elif key == "enter":
                seq += 1
                start = time.time()
                try:
                    self.motion.turn(token)
                    note = "ok"
                    fields = {"token": name}
                    self.hw.beep_ok()
                except Exception as exc:
                    note = "error"
                    fields = {"token": name, "err": repr(exc)}
                    self.hw.beep_error()
                fields["elapsed"] = round(time.time() - start, 3)
                self.log.write("DEBUG", "TURN_REPEAT", run_id, seq, fields, note)

    def debug_grip_jog(self, run_id):
        tags = ["close", "lift", "release", "free"]
        tag_i = 0
        seq = 0
        start_pos = self.grip_position()
        while True:
            state = self.buttons.state()
            speed = 0
            if state["left"] and state["right"]:
                speed = 0  # chord in progress -> hold still, let L+R exit
            elif state["right"]:
                speed = abs(config.GRIP_SPEED)
            elif state["left"]:
                speed = -abs(config.GRIP_SPEED)
            self.grip_on(speed)
            pos = self.grip_position()
            delta = pos - start_pos
            self.screen.show([
                "GRIP JOG",
                "R=ccw L=cw",
                "delta={}".format(delta),
                "tag={}".format(tags[tag_i]),
                "ENTER=snap",
                "BACK=menu",
            ])
            key = self.buttons.poll()
            if key == "back":
                self.hw.stop_grip()
                return
            if key == "up":
                tag_i = (tag_i - 1) % len(tags)
            elif key == "down":
                tag_i = (tag_i + 1) % len(tags)
            elif key == "enter":
                self.hw.stop_grip()
                seq += 1
                self.log.write("DEBUG", "GRIP_JOG", run_id, seq,
                               {"delta": delta, "tag": tags[tag_i]}, "snapshot")
                self.hw.beep_snapshot()
            time.sleep(0.03)

    def debug_grip_only(self, run_id):
        seq = 0
        while True:
            self.screen.show(["GRIP ONLY", "ENTER=run", "BACK=menu"])
            key = self.buttons.wait()
            if key == "back":
                return
            if key == "enter":
                seq += 1
                result = self.safe_grip_close_lift()
                self.log.write("DEBUG", "GRIP_ONLY", run_id, seq, result, "ok")
                self.hw.beep_ok()

    def debug_release_only(self, run_id):
        seq = 0
        while True:
            self.screen.show(["RELEASE ONLY", "ENTER=run", "BACK=menu"])
            key = self.buttons.wait()
            if key == "back":
                return
            if key == "enter":
                seq += 1
                result = self.safe_grip_release()
                self.log.write("DEBUG", "RELEASE_ONLY", run_id, seq, result, "ok")
                self.hw.beep_ok()

    def debug_distance_stream(self, run_id):
        seq = 0
        while True:
            dist = self.hw.distance_cm()
            detect = config.OBSTACLE_MIN_VALID_CM <= dist < config.OBSTACLE_DISTANCE_CM
            self.screen.show([
                "DIST STREAM",
                "cm={:.1f}".format(dist),
                "detect={}".format("YES" if detect else "NO"),
                "ENTER=snap",
                "BACK=menu",
            ])
            key = self.check_abort_edge()
            if key == "enter":
                seq += 1
                self.log.write("DEBUG", "DIST_STREAM", run_id, seq,
                               {"cm": round(dist, 2), "detect": detect}, "snapshot")
                self.hw.beep_snapshot()
            time.sleep(LIVE_SECONDS)

    def debug_color_repeat(self, run_id):
        seq = 0
        while True:
            self.screen.show(["COLOR REPEAT", "ENTER=read", "BACK=menu"])
            key = self.buttons.wait()
            if key == "back":
                return
            if key == "enter":
                seq += 1
                color = self.motion.read_node_color()
                self.log.write("DEBUG", "COLOR_REPEAT", run_id, seq,
                               {"color": color, "name": COLOR_NAMES.get(color, "?")},
                               "ok")
                self.hw.beep_snapshot()

    def confirm_debug_start(self, title):
        self.screen.show([title, "uses motors", "ENTER=start", "BACK=cancel"])
        while True:
            key = self.buttons.wait()
            if key == "back":
                raise UserAbort()
            if key == "enter":
                self.hw.beep_start()
                return

    # ------------------------------------------------------------------
    # GRIP HELPERS
    # ------------------------------------------------------------------
    def grip_position(self):
        try:
            return int(self.hw.grip_motor.position)
        except Exception:
            return 0

    def grip_on(self, speed):
        try:
            if speed:
                self.hw.grip_motor.on(self.hw._SpeedPercent(clamp(speed, -100, 100)))
            else:
                self.hw.stop_grip()
        except Exception:
            if speed == 0:
                return
            raise

    def run_grip_degrees(self, direction, degrees, speed):
        if degrees <= 0:
            return {"target": degrees, "delta": 0, "aborted": False, "guard": False}
        sign = 1 if direction == "ccw" else -1
        start_pos = self.grip_position()
        start_time = time.time()
        max_delta = abs(degrees) + 30
        timeout = min(3.0, max(0.5, abs(degrees) / float(max(speed, 1)) + 1.0))
        guard = False
        aborted = False
        self.grip_on(sign * abs(speed))
        while True:
            state = self.buttons.state()
            if state["enter"] or (state["left"] and state["right"]):
                aborted = True
                break
            pos = self.grip_position()
            delta = abs(pos - start_pos)
            self.screen.show([
                "GRIP MOVE",
                "dir={}".format(direction),
                "deg={}/{}".format(int(delta), degrees),
                "ENTER or L+R=stop",
            ])
            if delta >= abs(degrees):
                break
            if delta > max_delta or time.time() - start_time > timeout:
                guard = True
                break
            time.sleep(0.03)
        self.hw.stop_grip()
        actual = self.grip_position() - start_pos
        if aborted:
            raise UserAbort()
        if guard:
            self.hw.beep_error()
            raise RuntimeError("grip guard")
        return {"target": degrees, "delta": actual,
                "aborted": False, "guard": False}

    def safe_grip_close_lift(self):
        fields = {}
        close = self.run_grip_degrees("ccw", config.GRIP_CLOSE_DEGREES, config.GRIP_SPEED)
        fields["close_delta"] = close["delta"]
        if config.LIFT_DEGREES:
            lift = self.run_grip_degrees("ccw", config.LIFT_DEGREES, config.GRIP_SPEED)
            fields["lift_delta"] = lift["delta"]
        else:
            fields["lift_delta"] = 0
        self.hw._gripping = True
        if config.POST_GRIP_SETTLE_SECONDS:
            time.sleep(config.POST_GRIP_SETTLE_SECONDS)
        return fields

    def safe_grip_release(self):
        total = config.GRIP_CLOSE_DEGREES + (config.LIFT_DEGREES or 0)
        result = self.run_grip_degrees("cw", total, config.GRIP_SPEED)
        self.hw._gripping = False
        if config.POST_RELEASE_SETTLE_SECONDS:
            time.sleep(config.POST_RELEASE_SETTLE_SECONDS)
        return {"release_delta": result["delta"]}

    # ------------------------------------------------------------------
    # ADJUST
    # ------------------------------------------------------------------
    def run_adjust(self, item_index, run_id):
        group, name, base_step = ADJUST_ITEMS[item_index]
        step_options = self.step_options(base_step)
        step_i = 0
        old_value = getattr(config, name)
        value = self.adjust_start_value(name, old_value)
        seq = 0
        while True:
            step = step_options[step_i]
            self.screen.show([
                "ADJUST {}".format(group),
                name[:28],
                "old={}".format(format_value(old_value))[:28],
                "new={}".format(format_value(value))[:28],
                "step={}".format(step),
                "UD=val LR=step",
                "ENTER=save",
            ])
            key = self.buttons.wait()
            if key == "back":
                return
            if key == "left":
                step_i = (step_i - 1) % len(step_options)
            elif key == "right":
                step_i = (step_i + 1) % len(step_options)
            elif key in ("up", "down"):
                value = self.adjust_value(value, old_value, step, 1 if key == "up" else -1)
            elif key == "enter":
                seq += 1
                if not self.write_enabled:
                    self.log.write("ADJUST", "CONFIG_WRITE", run_id, seq,
                                   {"name": name, "old": old_value, "new": value},
                                   "write_disabled")
                    self.hw.beep_error()
                    self.show_pause(["WRITE DISABLED", "restart with --write"], 1.2)
                    continue
                if self.confirm_save(name, old_value, value):
                    ok = self.write_config_value(name, value)
                    note = "ok" if ok else "error"
                    self.log.write("ADJUST", "CONFIG_WRITE", run_id, seq,
                                   {"name": name, "old": old_value, "new": value},
                                   note)
                    if ok:
                        self.hw.beep_ok()
                        old_value = getattr(config, name)
                        value = self.adjust_start_value(name, old_value)
                    else:
                        self.hw.beep_error()

    def step_options(self, base):
        if isinstance(base, float) and base < 1:
            return [base, base * 5, base * 10]
        return [base, base * 5, base * 10]

    def adjust_start_value(self, name, old_value):
        if old_value is None and name.endswith("_LINE_THRESHOLD"):
            return config.LINE_THRESHOLD
        return old_value

    def adjust_value(self, value, old_value, step, direction):
        if isinstance(old_value, bool):
            return not bool(value)
        if value is None:
            value = 0
        value = value + step * direction
        if isinstance(old_value, int) and not isinstance(old_value, bool):
            return int(round(value))
        return round(float(value), 4)

    def confirm_save(self, name, old_value, value):
        self.screen.show([
            "SAVE CONFIG?",
            name[:28],
            "{} -> {}".format(format_value(old_value), format_value(value))[:28],
            "ENTER=yes",
            "BACK=no",
        ])
        while True:
            key = self.buttons.wait()
            if key == "back":
                return False
            if key == "enter":
                return True

    def write_config_value(self, name, value):
        old_value = getattr(config, name)
        try:
            setattr(config, name, value)
            config.validate_config()
        except Exception as exc:
            setattr(config, name, old_value)
            self.show_pause(["VALIDATE FAIL", str(exc)[:26]], 1.5)
            return False
        setattr(config, name, old_value)

        with open(CONFIG_PATH, "r", encoding="utf-8") as fp:
            lines = fp.readlines()
        pattern = re.compile(r"^(\s*{}\s*=\s*)([^#\n]*)(.*)$".format(re.escape(name)))
        replaced = False
        new_lines = []
        for line in lines:
            m = pattern.match(line)
            if m and not replaced:
                new_lines.append("{}{}{}\n".format(m.group(1), config_literal(value),
                                                    m.group(3).rstrip("\n")))
                replaced = True
            else:
                new_lines.append(line)
        if not replaced:
            self.show_pause(["WRITE FAIL", "name not found"], 1.2)
            return False
        try:
            shutil.copy2(CONFIG_PATH, CONFIG_BAK_PATH)
            with open(CONFIG_PATH, "w", encoding="utf-8") as fp:
                fp.writelines(new_lines)
            importlib.reload(config)
            config.validate_config()
            return True
        except Exception as exc:
            try:
                if os.path.exists(CONFIG_BAK_PATH):
                    shutil.copy2(CONFIG_BAK_PATH, CONFIG_PATH)
                    importlib.reload(config)
            except Exception:
                pass
            self.show_pause(["WRITE FAIL", str(exc)[:26]], 1.5)
            return False

    # ------------------------------------------------------------------
    # SYSTEM
    # ------------------------------------------------------------------
    def run_system(self, step, run_id):
        if step == "VALIDATE":
            try:
                config.validate_config()
                self.log.write("SYSTEM", "VALIDATE", run_id, 1, {}, "ok")
                self.hw.beep_ok()
                self.show_pause(["VALIDATE OK"], 1.0)
            except Exception as exc:
                self.log.write("SYSTEM", "VALIDATE", run_id, 1,
                               {"err": repr(exc)}, "error")
                self.hw.beep_error()
                self.show_pause(["VALIDATE FAIL", str(exc)[:26]], 1.5)
        elif step == "VIEW_CONFIG":
            self.view_core_config()
        elif step == "LOG_PATH":
            self.show_log_path()
        elif step == "BEEP_TEST":
            self.beep_test()
        elif step == "CONFIG_FINAL":
            self.log.write_config_final()
            self.hw.beep_ok()
            self.show_pause(["CONFIG_FINAL", "logged"], 1.0)
        elif step == "EXIT":
            self.exiting = True

    def view_core_config(self):
        pages = [
            ["THRESHOLD", "LINE={}".format(config.LINE_THRESHOLD),
             "LCR={}".format(config.line_thresholds())],
            ["FOLLOW", "speed={}".format(config.FOLLOW_SPEED),
             "KP={}".format(config.KP), "SIDE={}".format(config.SIDE_CORRECTION)],
            ["TRIM", "L={}".format(config.LEFT_MOTOR_TRIM),
             "R={}".format(config.RIGHT_MOTOR_TRIM)],
            ["TURN MIN", "L={}".format(config.LEFT_TURN_MIN_SECONDS),
             "R={}".format(config.RIGHT_TURN_MIN_SECONDS),
             "U={}".format(config.UTURN_MIN_SECONDS)],
            ["OBJECT", "dist={}".format(config.OBSTACLE_DISTANCE_CM),
             "grip={}".format(config.GRIP_CLOSE_DEGREES),
             "lift={}".format(config.LIFT_DEGREES)],
        ]
        page = 0
        while True:
            self.screen.show(pages[page] + ["LR=page", "BACK=menu"])
            key = self.buttons.wait()
            if key == "back":
                return
            if key == "left":
                page = (page - 1) % len(pages)
            elif key == "right" or key == "enter":
                page = (page + 1) % len(pages)

    def show_log_path(self):
        parts = [self.log.path[i:i + 26] for i in range(0, len(self.log.path), 26)]
        while True:
            self.screen.show(["LOG PATH"] + parts[:5] + ["BACK=menu"])
            if self.buttons.wait() == "back":
                return

    def beep_test(self):
        self.screen.show(["BEEP TEST"])
        self.hw.beep()
        time.sleep(0.2)
        self.hw.beep_snapshot()
        time.sleep(0.2)
        self.hw.beep_start()
        time.sleep(0.2)
        self.hw.beep_abort()
        time.sleep(0.2)
        self.hw.beep_error()
        self.log.write("SYSTEM", "BEEP_TEST", self.run_id, 1, {}, "ok")


def parse_args(argv):
    parser = argparse.ArgumentParser(description="EV3 on-brick calibrator")
    parser.add_argument("--write", action="store_true",
                        help="enable confirmed writes to robot/config.py")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    try:
        config.validate_config()
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 2
    try:
        from hardware import Ev3Hardware
        hw = Ev3Hardware()
    except Exception as exc:
        print("EV3 hardware init failed: {}".format(exc), file=sys.stderr)
        return 2
    try:
        Calibrator(hw, write_enabled=args.write).run()
    except KeyboardInterrupt:
        try:
            hw.stop_all()
        except Exception:
            pass
        print("aborted", file=sys.stderr)
        return 130
    return 0


if __name__ == "__main__":
    sys.exit(main())
