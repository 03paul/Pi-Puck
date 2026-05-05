import curses
import time
from pipuck.pipuck import PiPuck

# =========================
# CONFIG
# =========================

NORMAL_SPEED = 600
BOOST_SPEED = 1000
TURN_SPEED_NORMAL = 500
TURN_SPEED_BOOST = 900

# Wie lange ein Tastendruck "aktiv" bleibt
KEY_TIMEOUT = 0.15

LOOP_DELAY = 0.03

pipuck = PiPuck(epuck_version=2)


def clamp_speed(v):
    return max(-1024, min(1024, int(v)))


def set_motors(left, right):
    pipuck.epuck.set_motor_speeds(
        clamp_speed(left),
        clamp_speed(right)
    )


def stop():
    set_motors(0, 0)


def main(stdscr):
    curses.cbreak()
    curses.noecho()
    stdscr.nodelay(True)
    stdscr.keypad(True)

    last_move_key = None
    last_move_time = 0

    last_space_time = 0

    stdscr.clear()
    stdscr.addstr(0, 0, "Pi-Puck Manual Control")
    stdscr.addstr(2, 0, "W = forward")
    stdscr.addstr(3, 0, "S = backward")
    stdscr.addstr(4, 0, "A = left")
    stdscr.addstr(5, 0, "D = right")
    stdscr.addstr(6, 0, "SPACE = boost while active")
    stdscr.addstr(7, 0, "Q = quit")
    stdscr.addstr(9, 0, "Robot moves only while key is actively repeated.")
    stdscr.refresh()

    stop()

    try:
        while True:
            now = time.time()

            key = stdscr.getch()

            while key != -1:
                ch = chr(key).lower() if 0 <= key <= 255 else ""

                if ch in ["w", "a", "s", "d"]:
                    last_move_key = ch
                    last_move_time = now

                elif ch == " ":
                    last_space_time = now

                elif ch == "q":
                    stop()
                    return

                key = stdscr.getch()

            move_active = (now - last_move_time) <= KEY_TIMEOUT
            boost_active = (now - last_space_time) <= KEY_TIMEOUT

            speed = BOOST_SPEED if boost_active else NORMAL_SPEED
            turn_speed = TURN_SPEED_BOOST if boost_active else TURN_SPEED_NORMAL

            if not move_active:
                stop()
                status = "STOP"
            else:
                if last_move_key == "w":
                    set_motors(speed, speed)
                    status = "FORWARD BOOST" if boost_active else "FORWARD"

                elif last_move_key == "s":
                    set_motors(-speed, -speed)
                    status = "BACKWARD BOOST" if boost_active else "BACKWARD"

                elif last_move_key == "a":
                    set_motors(-turn_speed, turn_speed)
                    status = "LEFT BOOST" if boost_active else "LEFT"

                elif last_move_key == "d":
                    set_motors(turn_speed, -turn_speed)
                    status = "RIGHT BOOST" if boost_active else "RIGHT"

                else:
                    stop()
                    status = "STOP"

            stdscr.addstr(11, 0, " " * 60)
            stdscr.addstr(
                11,
                0,
                f"Status: {status} | speed={speed if boost_active else NORMAL_SPEED}"
            )
            stdscr.refresh()

            time.sleep(LOOP_DELAY)

    finally:
        stop()


if __name__ == "__main__":
    try:
        curses.wrapper(main)
    finally:
        stop()