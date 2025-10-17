import Jetson.GPIO as GPIO
import time

#GPIO.setwarnings(False)
# Set the pin numbering mode to BOARD (physical pin numbers)
GPIO.setmode(GPIO.BOARD)

# Define the pin connected to the LED (e.g., physical pin 7)
led_pin = 7

# Set up the pin as an output
GPIO.setup(led_pin, GPIO.OUT, initial=GPIO.LOW)

try:
    while True:
        # Turn LED on
        GPIO.output(led_pin, GPIO.HIGH)
        time.sleep(1)  # Wait for 500 milliseconds
        print("Blink")

        # Turn LED off
        GPIO.output(led_pin, GPIO.LOW)
        time.sleep(1)  # Wait for 500 milliseconds
        print("Blank")
except KeyboardInterrupt:
    # Clean up GPIO settings on exit
    GPIO.cleanup()
    print("Program stopped and GPIO cleaned up.")
