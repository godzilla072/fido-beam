from math import sin, cos
from pylx16a.lx16a import *
import time
import serial.serialutil

print("Hello World!")

# LX16A.initialize("/dev/ttyUSB0", 0.1)  # Linux
LX16A.initialize("/dev/cu.usbserial-120", 0.1)  # macOS

# A function to handle disconnection errors
def handle_disconnection(func):
    def wrapper(*args, **kwargs):
        try:
            return func(*args, **kwargs)
        except ServoTimeoutError as e:
            print(f"Servo {e.id_} is not responding. Exiting...")
            quit()
        except ServoChecksumError:
            print("Checksum error occurred while communicating with the servo. Exiting...")
            quit()
        except serial.serialutil.SerialException:
            print("Serial port error. The motor might be disconnected. Exiting...")
            quit()
        except Exception as e:
            print(f"An unexpected error occurred: {str(e)}. Exiting...")
            quit()
    return wrapper

try:
    servo1 = LX16A(3)  # Bottom
    servo2 = LX16A(4)  # Top
    
    servo1.set_angle_limits(155, 165)
    servo2.set_angle_limits(95, 120)
except ServoTimeoutError as e:
    print(f"Servo {e.id_} is not responding. Exiting...")
    quit()

t = 0

@handle_disconnection
def move_servos():
    global t
    while True:
        servo1.move(sin(t) * 60 + 60)
        servo2.move(cos(t) * 60 + 60)

        time.sleep(0.05)
        t += 0.1

move_servos()