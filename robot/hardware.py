#!/usr/bin/env python3
"""하드웨어 추상화 계층 (ev3dev2).

solver.py 는 모터/센서를 직접 다루지 않고 이 Ev3Hardware 인터페이스만 호출한다.
이렇게 분리하면 (1) 알고리즘 코드가 깔끔해지고, (2) 오프라인 검증용 가짜 하드웨어
(tests/sim_hardware.py)를 같은 인터페이스로 끼워 넣어 ev3dev 없이도 로직을 돌려볼 수 있다.

ev3dev2 임포트는 이 클래스를 생성할 때만 일어난다. 따라서 main.py 의 --dry-run 처럼
하드웨어가 없는 환경에서는 이 파일을 import 만 하고 Ev3Hardware() 를 만들지 않으면 된다.
"""

from __future__ import print_function

import time

import config


def clamp(value, low, high):
    return max(low, min(high, value))


# 중앙 컬러센서의 모드 문자열 (ev3dev2)
_MODE_REFLECT = "COL-REFLECT"   # 반사광(0~100) — 라인추종용
_MODE_COLOR = "COL-COLOR"       # 색 번호(0~7) — 노드 색 판정용


class Ev3Hardware(object):
    """실제 EV3 브릭 입출력을 감싼다."""

    def __init__(self):
        from ev3dev2.button import Button
        from ev3dev2.motor import LargeMotor, MediumMotor, SpeedPercent
        from ev3dev2.sensor.lego import ColorSensor, UltrasonicSensor
        from ev3dev2.sound import Sound

        self._SpeedPercent = SpeedPercent

        # 주행 모터 (왼쪽 A, 오른쪽 B)
        self.left_motor = LargeMotor(config.DRIVE_LEFT_PORT)
        self.right_motor = LargeMotor(config.DRIVE_RIGHT_PORT)
        # 그립&리프트 모터 (C)
        self.grip_motor = MediumMotor(config.GRIP_MOTOR_PORT)

        # 컬러센서 (좌/중/우). 중앙은 평소 반사광 모드로 둔다.
        self.left_sensor = ColorSensor(config.COLOR_LEFT_PORT)
        self.center_sensor = ColorSensor(config.COLOR_CENTER_PORT)
        self.right_sensor = ColorSensor(config.COLOR_RIGHT_PORT)
        self.left_sensor.mode = _MODE_REFLECT
        self.center_sensor.mode = _MODE_REFLECT
        self.right_sensor.mode = _MODE_REFLECT

        # 초음파 (4)
        self.ultrasonic = UltrasonicSensor(config.ULTRASONIC_PORT)

        self.buttons = Button()
        self.sound = Sound()
        self._gripping = False

    # ---- 주행 모터 -------------------------------------------------------
    def drive(self, left_speed, right_speed, apply_trim=True):
        """좌/우 바퀴 속도를 각각 준다(탱크 방식). 양수=전진.

        좌우 모터 편차는 config 의 곱셈 트림(LEFT/RIGHT_MOTOR_TRIM)으로 보정한다.
        직진 명령(좌==우)에도 한쪽으로 흐르면 트림으로 잡는다. 단, 제자리 회전은
        apply_trim=False 로 호출해 트림이 회전 거동(중심·균형)을 바꾸지 않게 한다.
        """
        if apply_trim:
            left_speed *= config.LEFT_MOTOR_TRIM
            right_speed *= config.RIGHT_MOTOR_TRIM
        self.left_motor.on(self._SpeedPercent(clamp(left_speed, -100, 100)))
        self.right_motor.on(self._SpeedPercent(clamp(right_speed, -100, 100)))

    def stop(self):
        self.left_motor.off(brake=True)
        self.right_motor.off(brake=True)

    def stop_drive(self):
        self.stop()

    # ---- 컬러센서 --------------------------------------------------------
    def read_reflect(self):
        """좌/중/우 반사광 3개를 (left, center, right) 튜플로 반환."""
        return (
            self.left_sensor.reflected_light_intensity,
            self.center_sensor.reflected_light_intensity,
            self.right_sensor.reflected_light_intensity,
        )

    def read_center_color(self):
        """중앙센서를 잠깐 컬러 모드로 바꿔 노드 색 번호(0~7)를 읽고 다시 반사광 모드로 복귀.

        막다른 길에서 멈춰 있을 때만 호출한다(모드 전환은 약간 느리므로 라인추종 중엔 쓰지 않음).
        모드 전환 직후 값이 튀므로 settle sleep + 더미 읽기로 안정화한 뒤 읽는다.
        반사광 모드로 되돌아온 뒤에도 라인추종 재개 전 짧게 settle 한다.
        """
        self.center_sensor.mode = _MODE_COLOR
        try:
            if config.COLOR_MODE_SETTLE_SECONDS:
                time.sleep(config.COLOR_MODE_SETTLE_SECONDS)
            for _ in range(config.COLOR_DUMMY_READS):
                _ = self.center_sensor.color        # 전환 직후 튀는 값은 버린다
            return self.center_sensor.color
        finally:
            self.center_sensor.mode = _MODE_REFLECT
            if config.COLOR_MODE_RESTORE_SETTLE_SECONDS:
                time.sleep(config.COLOR_MODE_RESTORE_SETTLE_SECONDS)

    # ---- 초음파 ----------------------------------------------------------
    def distance_cm(self):
        """전방 거리(cm). 측정 불가 시 큰 값."""
        return self.ultrasonic.distance_centimeters

    # ---- 그립&리프트 -----------------------------------------------------
    def grip(self):
        """집게를 닫아 물체를 집고(설정 시 살짝 들어 올림)."""
        if self._gripping:
            return
        self.grip_motor.on_for_degrees(config.GRIP_SPEED, config.GRIP_CLOSE_DEGREES)
        if config.LIFT_DEGREES:
            self.grip_motor.on_for_degrees(config.GRIP_SPEED, config.LIFT_DEGREES)
        self._gripping = True

    def release(self):
        """집게를 열어 물체를 내려놓는다(grip 의 역동작)."""
        if not self._gripping:
            return
        total = config.GRIP_CLOSE_DEGREES + (config.LIFT_DEGREES or 0)
        self.grip_motor.on_for_degrees(config.GRIP_SPEED, -total)
        self._gripping = False

    def stop_grip(self):
        self.grip_motor.off(brake=True)

    def stop_all(self):
        self.stop_drive()
        self.stop_grip()

    @property
    def holding_object(self):
        return self._gripping

    # ---- 버튼/소리 -------------------------------------------------------
    def button_state(self):
        b = self.buttons
        return {
            "up": bool(b.up),
            "down": bool(b.down),
            "left": bool(b.left),
            "right": bool(b.right),
            "enter": bool(b.enter),
            "back": bool(b.backspace),
        }

    def beep(self):
        try:
            self.sound.beep()
        except Exception:
            pass

    def _tone(self, freq, duration):
        try:
            self.sound.tone(freq, duration)
        except Exception:
            pass

    def beep_ok(self):
        self.beep()
        time.sleep(0.05)
        self.beep()

    def beep_snapshot(self):
        self._tone(880, 90)

    def beep_start(self):
        self._tone(220, 90)

    def beep_abort(self):
        self._tone(220, 90)
        time.sleep(0.05)
        self._tone(220, 90)

    def beep_error(self):
        self._tone(160, 350)

    def abort_requested(self):
        return bool(self.buttons.backspace)

    def enter_pressed(self):
        return bool(self.buttons.enter)
