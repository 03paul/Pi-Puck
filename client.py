import sys
import tty
import termios
import select
import time
from pipuck.pipuck import PiPuck

# =========================
# CONFIG
# =========================

NORMAL_FORWARD_SPEED = 600
NORMAL_TURN_SPEED = 500

BOOST_FORWARD_SPEED = 1000
BOOST_TURN_SPEED = 900

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


def forward(boost=False):
    speed = BOOST_FORWARD_SPEED if boost else NORMAL_FORWARD_SPEED
    set_motors(speed, speed)


def backward(boost=False):
    speed = BOOST_FORWARD_SPEED if boost else NORMAL_FORWARD_SPEED
    set_motors(-speed, -speed)


def turn_left(boost=False):
    speed = BOOST_TURN_SPEED if boost else NORMAL_TURN_SPEED
    set_motors(-speed, speed)


def turn_right(boost=False):
    speed = BOOST_TURN_SPEED if boost else NORMAL_TURN_SPEED
    set_motors(speed, -speed)


# =========================
# KEYBOARD INPUT
# =========================

def key_pressed():
    dr, _, _ = select.select([sys.stdin], [], [], 0)
    if dr:
        return sys.stdin.read(1)
    return None


# =========================
# MAIN
# =========================

old_settings = termios.tcgetattr(sys.stdin)

boost = False
current_action = "STOP"

try:
    tty.setcbreak(sys.stdin.fileno())

    print("Manual Pi-Puck control started.")
    print("W = forward | S = backward | A = left | D = right | X = stop | SPACE = toggle boost | Q = quit")
    print("Boost OFF")

    stop()

    while True:
        key = key_pressed()

        if key is not None:
            key = key.lower()

            if key == " ":
                boost = not boost
                print("Boost ON" if boost else "Boost OFF")

                # aktuelle Bewegung direkt mit neuer Boost-Stufe aktualisieren
                if current_action == "FORWARD":
                    forward(boost)
                elif current_action == "BACKWARD":
                    backward(boost)
                elif current_action == "LEFT":
                    turn_left(boost)
                elif current_action == "RIGHT":
                    turn_right(boost)
                else:
                    stop()

            elif key == "w":
                current_action = "FORWARD"
                print("Forward", "| BOOST" if boost else "")
                forward(boost)

            elif key == "s":
                current_action = "BACKWARD"
                print("Backward", "| BOOST" if boost else "")
                backward(boost)

            elif key == "a":
                current_action = "LEFT"
                print("Left", "| BOOST" if boost else "")
                turn_left(boost)

            elif key == "d":
                current_action = "RIGHT"
                print("Right", "| BOOST" if boost else "")
                turn_right(boost)

            elif key == "x":
                current_action = "STOP"
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