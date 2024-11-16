from math import sin, cos
from pylx16a.lx16a import LX16A, ServoTimeoutError
import time

# Constants
SERIAL_PORT = "/dev/ttyUSB0"
SERVO_IDS = [1, 2, 3, 4, 5, 6, 7, 8]
NEUTRAL_POSITION = 60
LIFTED_POSITION = 90
AMPLITUDE = 60
OFFSET = 60
FREQUENCY = 0.1


def boot_sequence():
    """Initialize servos and set angle limits."""
    LX16A.initialize(SERIAL_PORT, 0.1)
    servos = []
    try:
        for servo_id in SERVO_IDS:
            servo = LX16A(servo_id)
            servo.set_angle_limits(0, 240)
            servos.append(servo)
    except ServoTimeoutError as e:
        print(f"Servo {e.id_} is not responding. Exiting...")
        quit()
    return servos

def homing_sequence(servos):
    """Perform the homing sequence to bring servos to neutral positions."""
    print("Starting homing sequence...")
    # Front left leg (servos 1 and 2)
    servos[0].move(LIFTED_POSITION)
    servos[1].move(LIFTED_POSITION)
    time.sleep(0.5) 
    servos[0].move(NEUTRAL_POSITION)
    servos[1].move(NEUTRAL_POSITION)
    time.sleep(0.5)

    # Front right leg (servos 3 and 4)
    servos[2].move(LIFTED_POSITION)
    servos[3].move(LIFTED_POSITION)
    time.sleep(0.5)
    servos[2].move(NEUTRAL_POSITION)
    servos[3].move(NEUTRAL_POSITION)
    time.sleep(0.5)

    # Back left leg (servos 5 and 6)
    servos[4].move(LIFTED_POSITION)
    servos[5].move(LIFTED_POSITION)
    time.sleep(0.5)
    servos[4].move(NEUTRAL_POSITION)
    servos[5].move(NEUTRAL_POSITION)
    time.sleep(0.5)

    # Back right leg (servos 7 and 8)
    servos[6].move(LIFTED_POSITION)
    servos[7].move(LIFTED_POSITION)
    time.sleep(0.5)
    servos[6].move(NEUTRAL_POSITION)
    servos[7].move(NEUTRAL_POSITION)
    time.sleep(0.5)

    print("Homing sequence completed.")

def main():
    servos = boot_sequence()
    homing_sequence(servos)
    
    t = 0  # Time variable for smooth movement

    print("Starting main walking loop...")
    while True:
        # Front left leg (servos 1 and 2)
        servos[0].move(sin(t) * AMPLITUDE + OFFSET)
        servos[1].move(cos(t) * AMPLITUDE + OFFSET)

        # Back right leg (servos 7 and 8) - phase shifted
        servos[6].move(sin(t + 4.71) * AMPLITUDE + OFFSET)
        servos[7].move(cos(t + 4.71) * AMPLITUDE + OFFSET)

        # Front right leg (servos 3 and 4) - phase shifted for alternating gait
        servos[2].move(sin(t + 1.57) * AMPLITUDE + OFFSET)
        servos[3].move(cos(t + 1.57) * AMPLITUDE + OFFSET)

        # Back left leg (servos 5 and 6) - another phase shift
        servos[4].move(sin(t + 3.14) * AMPLITUDE + OFFSET)
        servos[5].move(cos(t + 3.14) * AMPLITUDE + OFFSET)

        # Increment time for smooth movement
        time.sleep(0.05)
        t += FREQUENCY

if __name__ == "__main__":
    main()
