from math import sin, cos, pi, ceil
from pylx16a.lx16a import *
import time
from datetime import datetime
import serial.serialutil
import threading

# Initialize the servo connection
LX16A.initialize("/dev/cu.usbserial-130", 0.1)  # macOS

# Servo configuration: ID and angle limits
# Note: The servo horns were installed without zeroing the servo, therefore each
# one may have a slightly different offset to achieve the same radial range of motion.
SERVOS = {
    1: {"name": "Back Left Bottom", "min_angle": 140.0, "max_angle": 190.0}, # A: as going to 140 is down
    2: {"name": "Back Left Top", "min_angle": 130.0, "max_angle": 130.0},

    3: {"name": "Front Left Bottom", "min_angle": 150.0, "max_angle": 200.0},  #  170 is down (clock-wise)
    4: {"name": "Front Left Top", "min_angle": 150.0, "max_angle": 150.0},

    5: {"name": "Front Right Bottom", "min_angle": 150.0, "max_angle": 200.0}, # A: increasing to 200 is bottom
    6: {"name": "Front Right Top", "min_angle": 110.0, "max_angle": 110.0},

    7: {"name": "Back Right Bottom", "min_angle": 140.0, "max_angle": 190.0}, # 190 is bottom
    8: {"name": "Back Right Top", "min_angle": 90.0, "max_angle": 90.0},    
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

def generate_angles(start, stop, step):
    """
    Generator to yield angles from start to stop with the given step.
    Handles both increasing and decreasing sequences.
    """
    if step == 0:
        raise ValueError("Step cannot be zero.")
    elif step > 0:
        current = start
        while current <= stop:
            yield current
            current += step
    else:
        current = start
        while current >= stop:
            yield current
            current += step

@handle_disconnection
def walk(stop_event):
    """Simulate a walking gait by moving servos periodically."""
    print("Starting walking loop. Press Ctrl+C to stop and return to homing.")
    t = 0  # Initialize global time variable
    delta_t = 0.1  # Time increment
    step_size = 5.0  # Degrees to move in each step
    delay = 0.5  # Seconds between steps for fine-tuning

    while not stop_event.is_set():
        for servo_id, config in SERVOS.items():
            if stop_event.is_set():
                break  # Exit early if stop_event is set during servo movements
            try:
                min_angle = config["min_angle"]
                max_angle = config["max_angle"]
                servo = config["servo"]

                # Determine the time variable for this servo
                # Group A: Front Left (IDs 3 and 4), Front Right (IDs 5 and 6)
                # Group B: Back Left (IDs 1 and 2), Back Right (IDs 7 and 8)
                # Note: Due to orientation clockwise on one side is counter-clockwise on the other.
                if servo_id in [3, 4, 5, 6]:  # Group A
                    time_var = t
                else:  # Group B
                    time_var = t + pi  # Phase shift by pi radians

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
            except Exception as e:
                print(f"Failed to move Servo {servo_id} ({config['name']}): {e}. Exiting...")
                stop_event.set()
                break

        # Increment the global time variable
        t += delta_t
        # Sleep to control the loop frequency
        time.sleep(0.2)  # TODO: Lower this value to increase speed

@handle_disconnection
def fine_tune_front_left_leg(stop_event):
    """Fine-tune the front left leg by lifting and lowering it."""
    print("Starting fine-tuning for Front Left Leg (Servos 3 and 4). Press Ctrl+C to stop.")
    servo_bottom = SERVOS[3]["servo"]  # Front Left Bottom
    servo_top = SERVOS[4]["servo"]     # Front Left Top

    # Define the range of motion for fine-tuning based on actual limits
    lift_angle_bottom = SERVOS[3]["max_angle"]  # Maximum angle to lift
    lower_angle_bottom = ceil(SERVOS[3]["min_angle"])  # Minimum angle to lower, rounded up to avoid exceeding

    lift_angle_top = SERVOS[4]["max_angle"]     # Maximum angle to lift
    lower_angle_top = SERVOS[4]["min_angle"]    # Minimum angle to lower

    step = 5.0  # Degrees to move in each step
    delay = 0.5  # Seconds between steps

    try:
        while not stop_event.is_set():
            # Lift the front left leg
            print("Lifting Front Left Leg...")
            for angle_bottom in generate_angles(lower_angle_bottom, lift_angle_bottom, step):
                if stop_event.is_set():
                    break
                try:
                    # Ensure angles do not exceed actual limits
                    angle_bottom = min(angle_bottom, SERVOS[3]["max_angle"])
                    angle_top = angle_bottom - 50  # Adjust based on mechanical design
                    angle_top = max(angle_top, SERVOS[4]["min_angle"])
                    angle_top = min(angle_top, SERVOS[4]["max_angle"])

                    servo_bottom.move(angle_bottom)
                    servo_top.move(angle_top)

                    current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                    print(f"[{current_time}] Servo 3 moved to {angle_bottom}°, Servo 4 moved to {angle_top}°")
                except Exception as e:
                    print(f"Error during fine-tuning: {e}. Exiting fine-tune mode.")
                    stop_event.set()
                    break

                time.sleep(delay)

            if stop_event.is_set():
                break

            # Lower the front left leg
            print("Lowering Front Left Leg...")
            for angle_bottom in generate_angles(lift_angle_bottom, lower_angle_bottom, -step):
                if stop_event.is_set():
                    break
                try:
                    # Ensure angles do not go below actual limits
                    angle_bottom = max(angle_bottom, SERVOS[3]["min_angle"])
                    angle_top = angle_bottom - 50  # Adjust based on mechanical design
                    angle_top = max(angle_top, SERVOS[4]["min_angle"])
                    angle_top = min(angle_top, SERVOS[4]["max_angle"])

                    servo_bottom.move(angle_bottom)
                    servo_top.move(angle_top)

                    current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                    print(f"[{current_time}] Servo 3 moved to {angle_bottom}°, Servo 4 moved to {angle_top}°")
                except Exception as e:
                    print(f"Error during fine-tuning: {e}. Exiting fine-tune mode.")
                    stop_event.set()
                    break

                time.sleep(delay)

    except Exception as e:
        print(f"Error during fine-tuning: {e}. Exiting fine-tune mode.")
        stop_event.set()

def main():
    boot_sequence()
    homing_sequence()

    while True:
        print("\nSelect Mode:")
        print("1. Start Walking")
        print("2. Fine-Tune Front Left Leg")
        print("3. Exit")
        choice = input("Enter your choice (1/2/3): ").strip()

        if choice == '1':
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
            print("Walking mode terminated gracefully.")

        elif choice == '2':
            # Create a threading.Event to signal stopping
            stop_event = threading.Event()
            # Start fine-tune in a separate thread
            fine_tune_thread = threading.Thread(target=fine_tune_front_left_leg, args=(stop_event,), daemon=True)
            fine_tune_thread.start()

            # Wait for KeyboardInterrupt to stop the fine-tune loop
            try:
                while fine_tune_thread.is_alive():
                    fine_tune_thread.join(timeout=1)
            except KeyboardInterrupt:
                print("\nKeyboardInterrupt detected. Terminating fine-tune loop...")
                stop_event.set()
                fine_tune_thread.join()

            homing_sequence()  # Call homing sequence after fine-tune loop ends
            print("Fine-tuning mode terminated gracefully.")

        elif choice == '3':
            print("Exiting program. Performing homing sequence...")
            homing_sequence()
            print("Program terminated gracefully.")
            break

        else:
            print("Invalid choice. Please enter 1, 2, or 3.")

if __name__ == "__main__":
    main()