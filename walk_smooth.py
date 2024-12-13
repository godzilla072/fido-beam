from math import sin, cos, pi
from pylx16a.lx16a import *
import time
from datetime import datetime
import serial.serialutil
import sys

# Initialize the servo connection
LX16A.initialize("/dev/cu.usbserial-130", 0.1)  # macOS

FREQUENCY = 1.5  # 0.5 Controls speed of walking

# Predefined range of motion offsets
BOTTOM_SWING_FORWARD_OFFSET = 40.0
TOP_LIFT_UP_OFFSET = 5.0

######### FRONT #########

FRONT_LEFT_BOTTOM = 170.0   # Decreasing this value makes the leg reach more forward
FRONT_LEFT_TOP = 190.0      # Increasing this value causes the leg to push more off the ground

FRONT_RIGHT_BOTTOM = 170.0  # Increasing this value makes the leg reach more forward
FRONT_RIGHT_TOP = 175.0     # Decreasing this value causes the leg to push more off the ground

######### BACK #########

BACK_LEFT_BOTTOM = 180.0    # Decreasing this value makes the leg reach more forward
BACK_LEFT_TOP = 180.0       # Increasing this value causes the leg to push more off the ground

BACK_RIGHT_BOTTOM = 170.0   # Increasing this value makes the leg reach more forward
BACK_RIGHT_TOP = 165.0      # Decreasing this value causes the leg to push more off the ground



# Servo configuration: ID and angle settings
SERVOS = {
    # BACK LEFT
    1: {
        "name": "Back Left Bottom",
        "neutral_angle": BACK_LEFT_BOTTOM,
        "type": "bottom",
        "movement_direction": "decrease"  # swing_forward = neutral_angle - offset
    },
    2: {
        "name": "Back Left Top",
        "neutral_angle": BACK_LEFT_TOP,
        "type": "top",
        "movement_direction": "decrease"  # lift_up = neutral_angle - offset
    },
    # BACK - RIGHT
    7: {
        "name": "Back Right Bottom",
        "neutral_angle": BACK_RIGHT_BOTTOM,
        "type": "bottom",
        "movement_direction": "increase"  # swing_forward = neutral_angle + offset
    },
    8: {
        "name": "Back Right Top",
        "neutral_angle": BACK_RIGHT_TOP,
        "type": "top",
        "movement_direction": "increase"  # lift_up = neutral_angle + offset
    },

    # FRONT - LEFT
    3: {
        "name": "Front Left Bottom",
        "neutral_angle": FRONT_LEFT_BOTTOM,
        "type": "bottom",
        "movement_direction": "decrease"  # swing_forward = neutral_angle - offset
    },
    4: {
        "name": "Front Left Top",
        "neutral_angle": FRONT_LEFT_TOP,
        "type": "top",
        "movement_direction": "increase"  # lift_up = neutral_angle + offset
    },
    # FRONT - RIGHT
    5: {
        "name": "Front Right Bottom",
        "neutral_angle": FRONT_RIGHT_BOTTOM,
        "type": "bottom",
        "movement_direction": "increase"  # swing_forward = neutral_angle + offset
    },
    6: {
        "name": "Front Right Top",
        "neutral_angle": FRONT_RIGHT_TOP,
        "type": "top",
        "movement_direction": "increase"  # lift_up = neutral_angle + offset
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

def configure_servos(servos):
    """
    Calculate and set swing and lift angles based on neutral_angle and movement_direction.
    Also calculate min_angle and max_angle for each servo.
    """
    for servo_id, config in servos.items():
        neutral = config["neutral_angle"]
        type_ = config["type"]
        direction = config["movement_direction"]

        if type_ == "bottom":
            if direction == "decrease":
                swing_forward = neutral - BOTTOM_SWING_FORWARD_OFFSET
                swing_backward = neutral + 0.1  # Minimal increase to prevent limit breach
            elif direction == "increase":
                swing_forward = neutral + BOTTOM_SWING_FORWARD_OFFSET
                swing_backward = neutral - 0.1  # Minimal decrease to prevent limit breach
            else:
                raise ValueError(f"Invalid movement_direction for Servo {servo_id}")

            # Assign calculated swing angles
            config["swing_forward"] = swing_forward
            config["swing_backward"] = swing_backward

            # Calculate min and max angles
            config["min_angle"] = min(swing_forward, swing_backward) - 20.0
            config["max_angle"] = max(swing_forward, swing_backward) + 20.0

        elif type_ == "top":
            if direction == "decrease":
                lift_up = neutral - TOP_LIFT_UP_OFFSET
                lift_down = neutral + 10.0  # Slight increase to prevent limit breach
            elif direction == "increase":
                lift_up = neutral + TOP_LIFT_UP_OFFSET
                lift_down = neutral - 0.1   # Slight decrease to prevent limit breach
            else:
                raise ValueError(f"Invalid movement_direction for Servo {servo_id}")

            # Assign calculated lift angles
            config["lift_up"] = lift_up
            config["lift_down"] = lift_down

            # Calculate min and max angles
            config["min_angle"] = min(lift_up, lift_down) - 20.0
            config["max_angle"] = max(lift_up, lift_down) + 20.0

        else:
            raise ValueError(f"Unknown servo type: {type_}")

@handle_disconnection
def configure_all_servos():
    """Configure servo angles and initialize them."""
    configure_servos(SERVOS)
    print("\nAll servos configured with calculated angles based on neutral positions.")

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
            if config["type"] == "bottom":
                config["swing_forward"] = clamp_angle(
                    servo_id, config["swing_forward"]
                )
                config["swing_backward"] = clamp_angle(
                    servo_id, config["swing_backward"]
                )
            elif config["type"] == "top":
                config["lift_up"] = clamp_angle(servo_id, config["lift_up"])
                config["lift_down"] = clamp_angle(servo_id, config["lift_down"])

            # Compute AMPLITUDE and OFFSET for smooth gait
            if 'swing_forward' in config and 'swing_backward' in config:
                swing_forward = config['swing_forward']
                swing_backward = config['swing_backward']
                AMPLITUDE = (swing_forward - swing_backward) / 2
                OFFSET = (swing_forward + swing_backward) / 2
                config['AMPLITUDE'] = AMPLITUDE
                config['OFFSET'] = OFFSET

            elif 'lift_up' in config and 'lift_down' in config:
                lift_up = config['lift_up']
                lift_down = config['lift_down']
                AMPLITUDE = (lift_up - lift_down) / 2
                OFFSET = (lift_up + lift_down) / 2
                config['AMPLITUDE'] = AMPLITUDE
                config['OFFSET'] = OFFSET

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

def walk_smoothly():
    """Animate walking process using sine and cosine functions."""
    print("\n### Starting smooth walking mode ###\n")

    t = 0
    delta_t = 0.05  # Time increment

    try:
        while True:
            # Dictionary to store current angles for all servos
            current_angles = {}

            for leg in LEGS:
                leg_name = leg['name']
                top_servo_id = leg['top_servo_id']
                bottom_servo_id = leg['bottom_servo_id']
                top_servo = SERVOS[top_servo_id]['servo']
                bottom_servo = SERVOS[bottom_servo_id]['servo']

                # Get AMPLITUDE and OFFSET for the servos
                top_amplitude = SERVOS[top_servo_id]['AMPLITUDE']
                top_offset = SERVOS[top_servo_id]['OFFSET']
                bottom_amplitude = SERVOS[bottom_servo_id]['AMPLITUDE']
                bottom_offset = SERVOS[bottom_servo_id]['OFFSET']

                # Assign phase shifts
                if leg_name in ['Front Left', 'Back Right']:
                    phase_shift = 0
                elif leg_name in ['Front Right', 'Back Left']:
                    phase_shift = pi
                else:
                    phase_shift = 0

                # Compute phase
                phase = t + phase_shift

                # Compute angles
                top_angle = top_amplitude * cos(phase) + top_offset  # Lift servos use cosine
                bottom_angle = bottom_amplitude * sin(phase) + bottom_offset  # Swing servos use sine

                # Clamp angles to ensure they are within limits
                top_angle_clamped = clamp_angle(top_servo_id, top_angle)
                bottom_angle_clamped = clamp_angle(bottom_servo_id, bottom_angle)

                # Move servos
                top_servo.move(top_angle_clamped)
                bottom_servo.move(bottom_angle_clamped)

                # Store current angles
                current_angles[top_servo_id] = top_angle_clamped
                current_angles[bottom_servo_id] = bottom_angle_clamped

            # Print current angles of all servos
            print("\n--- Current Servo Angles ---")
            for servo_id in sorted(SERVOS.keys()):
                servo_angle = current_angles.get(servo_id, SERVOS[servo_id]['neutral_angle'])
                servo_name = SERVOS[servo_id]['name']
                print(f"Servo {servo_id} ({servo_name}): {servo_angle:.2f}°")
            print("----------------------------\n")

            time.sleep(delta_t)
            t += FREQUENCY * delta_t
    except KeyboardInterrupt:
        print("\nWalking interrupted by user.")
    finally:
        # homing_sequence()
        # print("Robot returned to home position.")
        print("Stopped.")

@handle_disconnection
def configure_and_initialize_servos():
    """Configure servo angles and initialize them."""
    configure_all_servos()
    boot_sequence()

def main():
    configure_and_initialize_servos()
    homing_sequence()

    input("\nPress Enter to start smooth walking...\n")

    walk_smoothly()

if __name__ == "__main__":
    main()