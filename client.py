import sys
import tty
import termios
import select
import time
import math
from pipuck.pipuck import PiPuck

# =========================
# CONFIG
# =========================

NORMAL_FORWARD_SPEED = 600
NORMAL_TURN_SPEED = 500

BOOST_FORWARD_SPEED = 1000
BOOST_TURN_SPEED = 900

LOOP_DELAY = 0.05

robot_positions = {}

pipuck = PiPuck(epuck_version=2)

MY_ID = 38
TARGET_ID = 44


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

def normalize_angle(angle):
    return angle % 360

def angle_diff(target, current):
    """
    Differenz zwischen Zielwinkel und aktuellem Winkel.
    Ergebnis: -180 bis +180 Grad.
    """
    return (target - current + 180) % 360 - 180

def angle_to_target(robot, target):
    dx = target["x"] - robot["x"]
    dy = target["y"] - robot["y"]

    target_angle = math.degrees(math.atan2(dy, dx))
    return normalize_angle(target_angle)

def distance(robot, target):
    dx = target["x"] - robot["x"]
    dy = target["y"] - robot["y"]

    return math.sqrt(dx * dx + dy * dy)


# =========================
# KEYBOARD INPUT
# =========================

def key_pressed():
    dr, _, _ = select.select([sys.stdin], [], [], 0)
    if dr:
        return sys.stdin.read(1)
    return None

def get_state(rid):
    rid = str(rid)

    if rid not in robot_positions:
        return None

    data = robot_positions[rid]

    return {
        "x": data["position"][0],
        "y": data["position"][1],
        "angle": data.get("angle", None)
    }


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

        target_angle = angle_to_target(robot, obj)
        dist = distance(robot, obj)

        robot = get_state(MY_ID)
        obj = get_state(TARGET_ID)

        if robot["angle"] is not None:
            diff = angle_diff(target_angle, robot["angle"])

            print(
                f"Robot {MY_ID}: x={robot['x']:.3f}, y={robot['y']:.3f}, angle={robot['angle']:.1f}° | "
                f"Object {TARGET_ID}: x={obj['x']:.3f}, y={obj['y']:.3f} | "
                f"target_angle={target_angle:.1f}°, diff={diff:.1f}°, distance={dist:.3f}m"
            )

        else:
            print(
                f"Robot {MY_ID}: x={robot['x']:.3f}, y={robot['y']:.3f} | "
                f"Object {TARGET_ID}: x={obj['x']:.3f}, y={obj['y']:.3f} | "
                f"target_angle={target_angle:.1f}°, distance={dist:.3f}m"
            )

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