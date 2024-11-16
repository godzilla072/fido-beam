from math import sin, cos
from pylx16a.lx16a import *
import time
from datetime import datetime
import serial.serialutil
import threading

# Initialize the servo connection
LX16A.initialize("/dev/cu.usbserial-120", 0.1)  # macOS

# Servo configuration: ID and angle limits
# Note: The servo horns were installed without zeroing the servo, therefore each
# one may have a slightly different offset to achieve the same radial range of motion.
SERVOS = {
    1: {"name": "Back Left Bottom", "min_angle": 160.0, "max_angle": 190.0},
    2: {"name": "Back Left Top", "min_angle": 100.0, "max_angle": 160.0},
    
    3: {"name": "Front Left Bottom", "min_angle": 170.0, "max_angle": 200.0},
    4: {"name": "Front Left Top", "min_angle": 110.0, "max_angle": 170.0},
    
    5: {"name": "Front Right Bottom", "min_angle": 150.0, "max_angle": 180.0},
    6: {"name": "Front Right Top", "min_angle": 80.0, "max_angle": 140.0},
    
    7: {"name": "Back Right Bottom", "min_angle": 140.0, "max_angle": 170.0},
    8: {"name": "Back Right Top", "min_angle": 60.0, "max_angle": 120.0},  # Decrease, lowers leg
}

# A decorator to handle errors gracefully
def handle_disconnection(func):
    def wrapper(*args, **kwargs):
        try:
            return func(*args, **kwargs)
        except ServoTimeoutError as e:
            print(f"Servo {e.id_} is not responding. Exiting...")
            quit()
        except ServoChecksumError:
            print("Checksum error occurred while communicating with a servo. Exiting...")
            quit()
        except serial.serialutil.SerialException:
            print("Serial port error. The motor might be disconnected. Exiting...")
            quit()
        except Exception as e:
            print(f"An unexpected error occurred: {str(e)}. Exiting...")
            quit()
    return wrapper

@handle_disconnection
def boot_sequence():
    """Initialize servos and set angle limits."""
    print("Initializing servos...")
    for servo_id, config in SERVOS.items():
        try:
            servo = LX16A(servo_id)
            servo.set_angle_limits(config["min_angle"], config["max_angle"])
            actual_min_angle, actual_max_angle = servo.get_angle_limits()
            config["servo"] = servo
            # Update configuration with actual enforced limits from the servo
            config["min_angle"] = actual_min_angle
            config["max_angle"] = actual_max_angle
            print(f"Servo {servo_id} ({config['name']}) initialized with actual limits: "
                  f"{actual_min_angle}° to {actual_max_angle}°")
        except Exception as e:
            print(f"Failed to initialize Servo {servo_id} ({config['name']}): {e}. Exiting...")
            quit()

@handle_disconnection
def homing_sequence():
    """Bring all servos to their neutral positions."""
    print("Starting homing sequence...")
    for servo_id, config in SERVOS.items():
        try:
            neutral_angle = (config["min_angle"] + config["max_angle"]) / 2
            config["servo"].move(neutral_angle)
            current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
            print(f"[{current_time}] Servo {servo_id} ({config['name']}) set to neutral position: {neutral_angle}°")
        except Exception as e:
            print(f"Failed to home Servo {servo_id} ({config['name']}): {e}. Exiting...")
            quit()
    print("Homing sequence completed.")

@handle_disconnection
def walk(stop_event):
    """Simulate a walking gait by moving servos periodically."""
    print("Starting walking loop. Press Ctrl+C to stop and return to homing.")
    time_variables = {servo_id: 0 for servo_id in SERVOS.keys()}  # Initialize time variables

    while not stop_event.is_set():
        for servo_id, config in SERVOS.items():
            if stop_event.is_set():
                break  # Exit early if stop_event is set during servo movements
            try:
                min_angle = config["min_angle"]
                max_angle = config["max_angle"]
                servo = config["servo"]
                time_var = time_variables[servo_id]

                # Calculate angles using sine for bottom servos and cosine for top servos
                if "Bottom" in config["name"]:
                    raw_angle = (sin(time_var) * 0.5 + 0.5) * (max_angle - min_angle) + min_angle
                else:
                    raw_angle = (cos(time_var) * 0.5 + 0.5) * (max_angle - min_angle) + min_angle

                # Clamp the angle to ensure it's within [min_angle, max_angle]
                clamped_angle = max(min_angle, min(raw_angle, max_angle))

                # Move the servo to the clamped angle
                servo.move(clamped_angle)

                # Log the movement
                current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                print(f"[{current_time}] Servo {servo_id} ({config['name']}) moved to {clamped_angle}°")

                # Update the time variable
                time_variables[servo_id] += 0.1
            except Exception as e:
                print(f"Failed to move Servo {servo_id} ({config['name']}): {e}. Exiting...")
                stop_event.set()
                break

        # Sleep to control the loop frequency
        time.sleep(0.2)

def main():
    boot_sequence()
    homing_sequence()

    # Create a threading.Event to signal stopping
    stop_event = threading.Event()
    # Start walk() in a separate thread and pass the stop_event
    walk_thread = threading.Thread(target=walk, args=(stop_event,), daemon=True)
    walk_thread.start()

    # Wait for KeyboardInterrupt to stop the walk loop
    try:
        while walk_thread.is_alive():
            walk_thread.join(timeout=1)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt detected. Terminating walk loop...")
        stop_event.set()
        walk_thread.join()

    homing_sequence()  # Call homing sequence after walking loop ends
    print("Program terminated gracefully.")

if __name__ == "__main__":
        main()