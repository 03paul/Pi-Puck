import sys
import tty
import termios
import select
import time
from pipuck.pipuck import PiPuck

# =========================
# CONFIG
# =========================

SPEED_FORWARD = 600
SPEED_TURN = 500

LOOP_DELAY = 0.05

pipuck = PiPuck(epuck_version=2)


# =========================
# MOTOR CONTROL
# =========================

def clamp_speed(v):
    return max(-1024, min(1024, int(v)))


def set_motors(left, right):
    pipuck.epuck.set_motor_speeds(
        clamp_speed(left),
        clamp_speed(right)
    )


def stop():
    set_motors(0, 0)


def forward():
    set_motors(SPEED_FORWARD, SPEED_FORWARD)


def backward():
    set_motors(-SPEED_FORWARD, -SPEED_FORWARD)


def turn_left():
    set_motors(-SPEED_TURN, SPEED_TURN)


def turn_right():
    set_motors(SPEED_TURN, -SPEED_TURN)


# =========================
# KEYBOARD INPUT
# =========================

def key_pressed():
    """
    Non-blocking key read from terminal.
    Returns one character or None.
    """
    dr, _, _ = select.select([sys.stdin], [], [], 0)
    if dr:
        return sys.stdin.read(1)
    return None


# =========================
# MAIN
# =========================

old_settings = termios.tcgetattr(sys.stdin)

try:
    tty.setcbreak(sys.stdin.fileno())

    print("Manual Pi-Puck control started.")
    print("W = forward | S = backward | A = left | D = right | X = stop | Q = quit")

    stop()

    while True:
        key = key_pressed()

        if key is not None:
            key = key.lower()

            if key == "w":
                print("Forward")
                forward()

            elif key == "s":
                print("Backward")
                backward()

            elif key == "a":
                print("Left")
                turn_left()

            elif key == "d":
                print("Right")
                turn_right()

            elif key == "x":
                print("Stop")
                stop()

            elif key == "q":
                print("Quit")
                break

        time.sleep(LOOP_DELAY)

except KeyboardInterrupt:
    print("\nKeyboardInterrupt")

finally:
    stop()
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    print("Robot stopped.")