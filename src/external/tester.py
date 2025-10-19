import time

import serial

# Adjust port name to your system (check with ls /dev/ttyACM*)
ser = serial.Serial('COM3', 115200, timeout=1)
time.sleep(2)  # Allow Arduino to reset

def send_controls(steering, throttle):
    """
    Send steering and throttle values as comma-separated string
    Example: "120,95\n"
    """
    cmd = f"{steering},{throttle}\n"
    ser.write(cmd.encode())

# Example: sweep
""" for angle in range(45, 125, 10):
    send_controls(angle, 90)
    time.sleep(0.5)
 """
for throttle in range(25, 45, 5):
    send_controls(90, throttle)
    time.sleep(1)

send_controls(90, 90)  # neutral

send_controls(90, 90)  # neutral

send_controls(90, 90)  # neutral
