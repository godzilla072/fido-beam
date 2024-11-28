from math import sin, cos, pi, ceil
from pylx16a.lx16a import *
import time
from datetime import datetime
import serial.serialutil
import threading
import sys

# Initialize the servo connection
LX16A.initialize("/dev/cu.usbserial-130", 0.1)  # macOS

# Servo configuration: ID and angle limits
# Note: Adjust 'swing_backward', 'swing_forward', 'lift_down', and 'lift_up' as per your robot's design
SERVOS = {
    1: {"name": "Back Left Bottom", "min_angle": 160.0, "max_angle": 210.0, "swing_backward": 210.0, "swing_forward": 160.0},
    2: {"name": "Back Left Top", "min_angle": 129.0, "max_angle": 135.0, "lift_down": 130.0, "lift_up": 130.0}, # Push down

    3: {"name": "Front Left Bottom", "min_angle": 150.0, "max_angle": 200.0, "swing_backward": 150.0, "swing_forward": 200.0},
    4: {"name": "Front Left Top", "min_angle": 159.0, "max_angle": 165.0, "lift_down": 160.0, "lift_up": 160.0},

    5: {"name": "Front Right Bottom", "min_angle": 150.0, "max_angle": 205.0, "swing_backward": 200.0, "swing_forward": 150.0},
    6: {"name": "Front Right Top", "min_angle": 95.0, "max_angle": 100.0, "lift_down": 100.0, "lift_up": 100.0}, # lower is down

    7: {"name": "Back Right Bottom", "min_angle": 130.0, "max_angle": 180.0, "swing_backward": 180.0, "swing_forward": 130.0},
    8: {"name": "Back Right Top", "min_angle": 69.0, "max_angle": 75.0, "lift_down": 70.0, "lift_up": 70.0},  # Push down, lower number
}

GROUP_A = [3, 4, 7, 8]  # Front Left and Front Right
GROUP_B = [1, 2, 5, 6]  # Back Left and Back Right

# Define custom exceptions if they are not part of pylx16a
# If pylx16a.lx16a already defines these, you can remove these definitions
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
            if threading.current_thread() == threading.main_thread():
                print("Exiting program due to servo timeout.")
                quit()
            else:
                print("Exiting thread due to servo timeout.")
        except ServoChecksumError:
            print("Checksum error occurred while communicating with a servo.")
            if threading.current_thread() == threading.main_thread():
                print("Exiting program due to checksum error.")
                quit()
            else:
                print("Exiting thread due to checksum error.")
        except serial.serialutil.SerialException:
            print("Serial port error. The motor might be disconnected.")
            if threading.current_thread() == threading.main_thread():
                print("Exiting program due to serial port error.")
                quit()
            else:
                print("Exiting thread due to serial port error.")
        except Exception as e:
            print(f"An unexpected error occurred: {str(e)}.")
            if threading.current_thread() == threading.main_thread():
                print("Exiting program due to unexpected error.")
                quit()
            else:
                print("Exiting thread due to unexpected error.")
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
            raise  # Let the decorator handle the exception

@handle_disconnection
def homing_sequence():
    """Bring all servos to their neutral positions."""
    print("Starting homing sequence...")
    for servo_id, config in SERVOS.items():
        try:
            if "Bottom" in config["name"]:
                neutral_angle = (config["swing_backward"] + config["swing_forward"]) / 2
            else:
                neutral_angle = config["lift_down"]  # Legs down in neutral position
            config["servo"].move(neutral_angle)
            current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
            print(f"[{current_time}] Servo {servo_id} ({config['name']}) set to neutral position: {neutral_angle}°")
        except Exception as e:
            print(f"Failed to home Servo {servo_id} ({config['name']}): {e}. Exiting...")
            raise  # Let the decorator handle the exception
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
def lift_legs(group):
    """Lift the legs in the specified group."""
    for servo_id in group:
        config = SERVOS[servo_id]
        if "Top" in config["name"]:
            servo = config["servo"]
            lift_up_angle = config["lift_up"]
            servo.move(lift_up_angle)
            current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
            print(f"[{current_time}] Servo {servo_id} ({config['name']}) lifted to {lift_up_angle}°")

@handle_disconnection
def lower_legs(group):
    """Lower the legs in the specified group."""
    for servo_id in group:
        config = SERVOS[servo_id]
        if "Top" in config["name"]:
            servo = config["servo"]
            lift_down_angle = config["lift_down"]
            servo.move(lift_down_angle)
            current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
            print(f"[{current_time}] Servo {servo_id} ({config['name']}) lowered to {lift_down_angle}°")

@handle_disconnection
def swing_legs_forward(group, step_size, delay):
    """Swing the legs forward in the specified group."""
    angle_sequences = {}
    max_steps = 0

    # Generate angle sequences for each servo
    for servo_id in group:
        config = SERVOS[servo_id]
        if "Bottom" in config["name"]:
            swing_backward_angle = config["swing_backward"]
            swing_forward_angle = config["swing_forward"]
            if swing_backward_angle < swing_forward_angle:
                step = step_size
            else:
                step = -step_size
            angle_sequence = list(generate_angles(swing_backward_angle, swing_forward_angle, step))
            angle_sequences[servo_id] = angle_sequence
            if len(angle_sequence) > max_steps:
                max_steps = len(angle_sequence)

    # Move servos step by step
    for step in range(max_steps):
        for servo_id, angles in angle_sequences.items():
            if step < len(angles):
                angle = angles[step]
                config = SERVOS[servo_id]
                servo = config["servo"]
                servo.move(angle)
                current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                print(f"[{current_time}] Servo {servo_id} ({config['name']}) moved to {angle}°")
        time.sleep(delay)

@handle_disconnection
def walk(stop_event):
    """Simulate walking by moving groups alternately."""
    print("Starting walking loop. Press Ctrl+C to stop and return to homing.")
    step_size = 5.0  # Degrees to move in each step
    delay = 0.2  # Seconds between steps

    try:
        while not stop_event.is_set():
            # Move Group A
            if stop_event.is_set():
                break
            print("Group A is moving")
            lift_legs(GROUP_A)
            swing_legs_forward(GROUP_A, step_size, delay)
            lower_legs(GROUP_A)

            # Wait briefly
            time.sleep(1.5)

            # Move Group B
            if stop_event.is_set():
                break
            print("Group B is moving")
            lift_legs(GROUP_B)
            swing_legs_forward(GROUP_B, step_size, delay)
            lower_legs(GROUP_B)

            # Wait briefly
            time.sleep(1.5)
    except Exception as e:
        print(f"Error in walk function: {e}")
        # Do not call quit() here; the decorator handles it based on the thread

@handle_disconnection
def fine_tune_front_left_leg(stop_event):
    """Fine-tune the front left leg by lifting and lowering it."""
    print("Starting fine-tuning for Front Left Leg (Servos 3 and 4). Press Ctrl+C to stop.")
    servo_bottom = SERVOS[3]["servo"]  # Front Left Bottom
    servo_top = SERVOS[4]["servo"]     # Front Left Top

    # Define the range of motion for fine-tuning based on actual limits
    lift_angle_bottom = SERVOS[3]["max_angle"]  # Maximum angle to lift
    lower_angle_bottom = ceil(SERVOS[3]["min_angle"])  # Minimum angle to lower, rounded up to avoid exceeding

    lift_angle_top = SERVOS[4]["lift_up"]     # Maximum angle to lift
    lower_angle_top = SERVOS[4]["lift_down"]    # Minimum angle to lower

    step = 5.0  # Degrees to move in each step
    delay_time = 0.5  # Seconds between steps

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

                time.sleep(delay_time)

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

                time.sleep(delay_time)

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
        try:
            choice = input("Enter your choice (1/2/3): ").strip()
        except EOFError:
            print("\nEOF detected. Exiting program.")
            homing_sequence()
            break
        except ValueError as ve:
            print(f"Input error: {ve}. Please try again.")
            continue

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