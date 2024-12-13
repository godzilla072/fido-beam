from math import ceil
from pylx16a.lx16a import *
import time
from datetime import datetime
import serial.serialutil
import sys

# Initialize the servo connection
LX16A.initialize("/dev/cu.usbserial-1130", 0.1)  # macOS

# Servo configuration: ID and angle settings
SERVOS = {
    1: {
        "name": "Back Left Bottom",
        "min_angle": 160.0,
        "max_angle": 210.0,
        "neutral_angle": 185.0,
        "swing_forward": 140.1,   # Higher angle for back servos to swing forward - 150
        "swing_backward": 185.1    # Slightly above min_angle to prevent limit breach
    },
    2: {
        "name": "Back Left Top",
        "min_angle": 105.0,
        "max_angle": 135.0,
        "neutral_angle": 125.0,
        "lift_down": 125.9,        # Slightly above min_angle
        "lift_up": 115.1           # Slightly below max_angle
    },
    3: {
        "name": "Front Left Bottom",
        "min_angle": 130.0,
        "max_angle": 220.0,
        "neutral_angle": 210.0,
        "swing_forward": 170.0,    # Lower angle for front servos to swing forward - 180
        "swing_backward": 210.82    # Slightly below max_angle to prevent limit breach
    },
    4: {
        "name": "Front Left Top",
        "min_angle": 120.0,
        "max_angle": 180.0,
        "neutral_angle": 130.0,    # Centered neutral position
        "lift_down": 135.8,         # Slightly below max_angle
        "lift_up": 125.1           # Slightly above min_angle (inverted)
    },
    5: {
        "name": "Front Right Bottom",
        "min_angle": 150.0,
        "max_angle": 215.0,
        "neutral_angle": 170.0,
        "swing_forward": 209.82,    # Lower angle for front servos to swing forward - 199
        "swing_backward": 170.0    # Slightly below max_angle to prevent limit breach
    },
    6: {
        "name": "Front Right Top",
        "min_angle": 75.0,
        "max_angle": 135.0,
        "neutral_angle": 100.0,
        "lift_down": 90.78 ,      # Slightly above min_angle
        "lift_up": 110.1          # Slightly below max_angle
    },
    7: {
        "name": "Back Right Bottom",
        "min_angle": 130.0,
        "max_angle": 179.76,
        "neutral_angle": 160.0,
        "swing_forward": 200.66,   # Higher angle for back servos to swing forward - 190
        "swing_backward": 150.1     # Slightly above min_angle to prevent limit breach
    },
    8: {
        "name": "Back Right Top",
        "min_angle": 55.0,
        "max_angle": 90.0,
        "neutral_angle": 65.0,
        "lift_down": 55.1,          # Slightly above min_angle
        "lift_up": 75.78            # Slightly below max_angle
    },
}

# Define the legs and their servos
LEGS = [
    {'name': 'Front Left', 'top_servo_id': 4, 'bottom_servo_id': 3},
    {'name': 'Front Right', 'top_servo_id': 6, 'bottom_servo_id': 5},
    {'name': 'Back Left', 'top_servo_id': 2, 'bottom_servo_id': 1},
    {'name': 'Back Right', 'top_servo_id': 8, 'bottom_servo_id': 7},
]

# Define custom exceptions if they are not part of pylx16a
class ServoTimeoutError(Exception):
    def __init__(self, id_):
        self.id_ = id_
        super().__init__(f"Servo {id_} timed out.")

class ServoChecksumError(Exception):
    pass

# A decorator to handle errors gracefully
def handle_disconnection(func):
    def wrapper(*args, **kwargs):
        try:
            return func(*args, **kwargs)
        except ServoTimeoutError as e:
            print(f"Servo {e.id_} is not responding.")
            sys.exit(1)
        except ServoChecksumError:
            print("Checksum error occurred while communicating with a servo.")
            sys.exit(1)
        except serial.serialutil.SerialException:
            print("Serial port error. The motor might be disconnected.")
            sys.exit(1)
        except Exception as e:
            print(f"An unexpected error occurred: {str(e)}.")
            sys.exit(1)
    return wrapper

@handle_disconnection
def boot_sequence():
    """Initialize servos and set angle limits."""
    print("\nInitializing servos...")
    for servo_id, config in SERVOS.items():
        try:
            servo = LX16A(servo_id)
            servo.set_angle_limits(config["min_angle"], config["max_angle"])
            actual_min_angle, actual_max_angle = servo.get_angle_limits()
            config["servo"] = servo
            # Update configuration with actual enforced limits from the servo
            config["min_angle"] = actual_min_angle
            config["max_angle"] = actual_max_angle

            # Ensure swing and lift angles are within actual limits
            if "Bottom" in config["name"]:
                # For Front servos, swing_forward is lower angle (to swing forward)
                if config["name"] in ["Front Left Bottom", "Front Right Bottom"]:
                    # Ensure swing_forward is slightly above min_angle
                    config["swing_forward"] = clamp_angle(
                        servo_id, config["swing_forward"]
                    )
                    # Ensure swing_backward is slightly below max_angle
                    config["swing_backward"] = clamp_angle(
                        servo_id, config["swing_backward"]
                    )
                else:
                    # For Back servos, swing_forward is higher angle (to swing forward)
                    config["swing_forward"] = clamp_angle(
                        servo_id, config["swing_forward"]
                    )
                    config["swing_backward"] = clamp_angle(
                        servo_id, config["swing_backward"]
                    )
            else:
                # For Top servos, ensure lift_up and lift_down are within limits
                config["lift_down"] = clamp_angle(servo_id, config["lift_down"])
                config["lift_up"] = clamp_angle(servo_id, config["lift_up"])

            print(f"Servo {servo_id} ({config['name']}) initialized with actual limits: "
                  f"{actual_min_angle}° to {actual_max_angle}°")
        except Exception as e:
            print(f"Failed to initialize Servo {servo_id} ({config['name']}): {e}. Exiting...")
            raise  # Let the decorator handle the exception

@handle_disconnection
def homing_sequence():
    """Bring all servos to their neutral positions."""
    print("\nStarting homing sequence...")
    for servo_id, config in SERVOS.items():
        try:
            neutral_angle = config["neutral_angle"]
            # Clamp the neutral_angle within servo limits
            neutral_angle = clamp_angle(servo_id, neutral_angle)
            config["servo"].move(neutral_angle)
            current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
            print(f"[{current_time}] Servo {servo_id} ({config['name']}) set to neutral position: {neutral_angle}°")
        except Exception as e:
            print(f"Failed to home Servo {servo_id} ({config['name']}): {e}. Exiting...")
            raise  # Let the decorator handle the exception
    print("Homing sequence completed.")

def clamp_angle(servo_id, angle):
    """Clamp the angle to the servo's min and max limits."""
    min_angle = SERVOS[servo_id]["min_angle"]
    max_angle = SERVOS[servo_id]["max_angle"]
    clamped = max(min(angle, max_angle), min_angle)
    if clamped != angle:
        print(f"Warning: Angle {angle}° for Servo {servo_id} ({SERVOS[servo_id]['name']}) clamped to {clamped}° to stay within limits.")
    return clamped

def walk_step_by_step():
    """Animate walking process by moving legs in pairs."""
    print("\n### Starting step-by-step walking mode (Leg Pairs) ###\n")

    # Define leg pairs
    leg_pairs = [
        [{'name': 'Front Left', 'top_servo_id': 4, 'bottom_servo_id': 3},
         {'name': 'Back Right', 'top_servo_id': 8, 'bottom_servo_id': 7}],
        [{'name': 'Front Right', 'top_servo_id': 6, 'bottom_servo_id': 5},
         {'name': 'Back Left', 'top_servo_id': 2, 'bottom_servo_id': 1}],
    ]

    for pair in leg_pairs:
        print(f"\n### Moving leg pair: {pair[0]['name']} and {pair[1]['name']} ###\n")

        # Lift both legs
        for leg in pair:
            print(f"Lifting {leg['name']} leg.")
            top_servo_id = leg['top_servo_id']
            top_servo = SERVOS[top_servo_id]['servo']
            lift_up_angle = SERVOS[top_servo_id]['lift_up']
            lift_up_angle = clamp_angle(top_servo_id, lift_up_angle)
            top_servo.move(lift_up_angle)
            print(f"Top Servo {top_servo_id} moved to {lift_up_angle}°")

        input("\nPress Enter to proceed to swing both legs forward...\n")

        # Swing both legs forward
        for leg in pair:
            print(f"Swinging {leg['name']} leg forward.")
            bottom_servo_id = leg['bottom_servo_id']
            bottom_servo = SERVOS[bottom_servo_id]['servo']
            swing_forward_angle = SERVOS[bottom_servo_id]['swing_forward']
            swing_forward_angle = clamp_angle(bottom_servo_id, swing_forward_angle)
            bottom_servo.move(swing_forward_angle)
            print(f"Bottom Servo {bottom_servo_id} moved to {swing_forward_angle}°")

        input("\nPress Enter to proceed to lower both legs...\n")

        # Lower both legs
        for leg in pair:
            print(f"Lowering {leg['name']} leg.")
            top_servo_id = leg['top_servo_id']
            top_servo = SERVOS[top_servo_id]['servo']
            lift_down_angle = SERVOS[top_servo_id]['lift_down']
            lift_down_angle = clamp_angle(top_servo_id, lift_down_angle)
            top_servo.move(lift_down_angle)
            print(f"Top Servo {top_servo_id} moved to {lift_down_angle}°")

        input("\nPress Enter to proceed to swing both legs backward...\n")

        # Swing both legs backward to neutral position
        for leg in pair:
            print(f"Swinging {leg['name']} leg backward to neutral position.")
            bottom_servo_id = leg['bottom_servo_id']
            bottom_servo = SERVOS[bottom_servo_id]['servo']
            neutral_angle = SERVOS[bottom_servo_id]['neutral_angle']
            neutral_angle = clamp_angle(bottom_servo_id, neutral_angle)
            bottom_servo.move(neutral_angle)
            print(f"Bottom Servo {bottom_servo_id} moved to {neutral_angle}°")

        input("\nPress Enter to proceed to the next leg pair...\n")

    print("\n### Step-by-step walking mode completed ###")
    homing_sequence()
    print("Robot returned to home position.")


def main():
    boot_sequence()
    homing_sequence()

    input("\nPress Enter to start...\n")

    walk_step_by_step()

if __name__ == "__main__":
    main()