from pylx16a.lx16a import *
import time
from datetime import datetime
import sys
import serial

# Initialize the servo connection
LX16A.initialize("/dev/cu.usbserial-1130", 0.1)  # Adjust the port as needed for your system

# Servo configuration: ID and names
SERVOS = {
    1: {"name": "Back Left Bottom"},
    2: {"name": "Back Left Top"},
    3: {"name": "Front Left Bottom"},
    4: {"name": "Front Left Top"},
    5: {"name": "Front Right Bottom"},
    6: {"name": "Front Right Top"},
    7: {"name": "Back Right Bottom"},
    8: {"name": "Back Right Top"},
}

# Initialize servos and store them in the SERVOS dict
for servo_id in SERVOS:
    try:
        servo = LX16A(servo_id)
        # Override angle limits to allow full range of motion
        servo.set_angle_limits(0.0, 240.0)
        SERVOS[servo_id]["servo"] = servo
    except Exception as e:
        print(f"Failed to initialize Servo {servo_id}: {e}")
        sys.exit(1)

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

def print_current_angles():
    current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
    print(f"\n[{current_time}] Current servo angles:")
    for servo_id, config in SERVOS.items():
        servo = config.get("servo")
        if servo:
            try:
                angle = servo.get_physical_angle()
                print(f"Servo {servo_id} ({config['name']}): {angle:.2f}°")
            except Exception as e:
                print(f"Failed to get angle for Servo {servo_id}: {e}")
        else:
            print(f"Servo {servo_id} ({config['name']}) not initialized")

@handle_disconnection
def main():
    selected_servo_id = None
    while True:
        print_current_angles()
        print("\nMenu:")
        print("1. Select servo to control")
        print("2. Enter angle to move servo to")
        print("3. Record current angle as min angle")
        print("4. Record current angle as max angle")
        print("5. Record current angle as neutral angle")
        print("6. Quit")
        choice = input("Enter your choice: ")
        if choice == '1':
            servo_id_input = input("Enter servo ID to control (1-8): ")
            try:
                servo_id = int(servo_id_input)
                if servo_id in SERVOS:
                    selected_servo_id = servo_id
                    print(f"Selected Servo {servo_id} ({SERVOS[servo_id]['name']})")
                else:
                    print("Invalid servo ID.")
            except ValueError:
                print("Invalid input. Please enter a number between 1 and 8.")
        elif choice == '2':
            if selected_servo_id is None:
                servo_id_input = input("Enter servo ID to move (1-8): ")
                try:
                    servo_id = int(servo_id_input)
                    if servo_id in SERVOS:
                        selected_servo_id = servo_id
                    else:
                        print("Invalid servo ID.")
                        continue
                except ValueError:
                    print("Invalid input. Please enter a number between 1 and 8.")
                    continue
            else:
                servo_id = selected_servo_id

            angle_input = input("Enter angle to move servo to (0-240 degrees): ")
            try:
                angle = float(angle_input)
                if 0 <= angle <= 240:
                    # **Added Confirmation Prompt Below**
                    confirmation = input(f"Confirm moving Servo {servo_id} ({SERVOS[servo_id]['name']}) to {angle}°? Press Enter to confirm or type 'cancel' to abort: ")
                    if confirmation.lower() == 'cancel':
                        print("Operation cancelled.")
                        continue  # Skip moving the servo
                    # If confirmation is just Enter (empty string), proceed
                    servo = SERVOS[servo_id]["servo"]
                    servo.move(angle)
                    print(f"Moved Servo {servo_id} to {angle}°")
                else:
                    print("Invalid angle. Please enter a value between 0 and 240.")
            except ValueError:
                print("Invalid input. Please enter a number.")
            except Exception as e:
                print(f"Failed to move Servo {servo_id}: {e}")
        elif choice in ['3', '4', '5']:
            if selected_servo_id is None:
                servo_id_input = input("Enter servo ID to record angle for (1-8): ")
                try:
                    servo_id = int(servo_id_input)
                    if servo_id in SERVOS:
                        selected_servo_id = servo_id
                    else:
                        print("Invalid servo ID.")
                        continue
                except ValueError:
                    print("Invalid input. Please enter a number between 1 and 8.")
                    continue
            else:
                servo_id = selected_servo_id

            servo = SERVOS[servo_id]["servo"]
            try:
                angle = servo.get_physical_angle()
                if choice == '3':
                    SERVOS[servo_id]["min_angle"] = angle
                    print(f"Recorded min angle for Servo {servo_id} as {angle:.2f}°")
                elif choice == '4':
                    SERVOS[servo_id]["max_angle"] = angle
                    print(f"Recorded max angle for Servo {servo_id} as {angle:.2f}°")
                elif choice == '5':
                    SERVOS[servo_id]["neutral_angle"] = angle
                    print(f"Recorded neutral angle for Servo {servo_id} as {angle:.2f}°")
            except Exception as e:
                print(f"Failed to get angle for Servo {servo_id}: {e}")
        elif choice == '6':
            print("Exiting.")
            # Optionally, print out the collected min, max, and neutral angles
            print("\nCollected servo angle measurements:")
            for servo_id, config in SERVOS.items():
                print(f"Servo {servo_id} ({config['name']}):")
                min_angle = config.get("min_angle", "Not recorded")
                max_angle = config.get("max_angle", "Not recorded")
                neutral_angle = config.get("neutral_angle", "Not recorded")
                print(f"  Min Angle: {min_angle}")
                print(f"  Max Angle: {max_angle}")
                print(f"  Neutral Angle: {neutral_angle}")
            break
        else:
            print("Invalid choice. Please select a number from 1 to 6.")
        time.sleep(0.5)

if __name__ == "__main__":
    main()