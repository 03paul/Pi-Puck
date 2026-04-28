import time
import random
from pipuck.pipuck import PiPuck

pipuck = PiPuck(epuck_version=2)

MAX_SPEED = 1000
FORWARD_MIN = 1.0
FORWARD_MAX = 3.0

SPIN_MIN = 0.4
SPIN_MAX = 1.2
SPIN_REPEATS_MIN = 2
SPIN_REPEATS_MAX = 5

def stop():
    pipuck.epuck.set_motor_speeds(0, 0)

def forward():
    pipuck.epuck.set_motor_speeds(MAX_SPEED, MAX_SPEED)

def spin_left():
    pipuck.epuck.set_motor_speeds(-MAX_SPEED, MAX_SPEED)

def spin_right():
    pipuck.epuck.set_motor_speeds(MAX_SPEED, -MAX_SPEED)

def random_turn():
    direction = random.choice(["left", "right"])
    duration = random.uniform(0.2, 1.0)

    if direction == "left":
        spin_left()
    else:
        spin_right()

    time.sleep(duration)

try:
    while True:
        # 1. Maximal schnell vorwärts
        forward()
        time.sleep(random.uniform(FORWARD_MIN, FORWARD_MAX))

        # 2. Disco-Spin: mehrmals schnell im Kreis drehen
        repeats = random.randint(SPIN_REPEATS_MIN, SPIN_REPEATS_MAX)

        for _ in range(repeats):
            if random.choice([True, False]):
                spin_left()
            else:
                spin_right()

            time.sleep(random.uniform(SPIN_MIN, SPIN_MAX))

        # 3. Random Richtung wählen
        random_turn()

        # Danach loop: wieder Vollgas geradeaus

except KeyboardInterrupt:
    print("Stopping robot...")

finally:
    stop()