from machine import Pin, PWM
import sys
import time

# ===== CONFIGURACIÓN PINES =====
STEERING_PIN = 9
THROTTLE_PIN = 10
CLAXON_PIN = 8

# ===== PWM SERVOS =====
steering_pwm = PWM(Pin(STEERING_PIN), freq=50)
throttle_pwm = PWM(Pin(THROTTLE_PIN), freq=50)

# ===== PWM CLAXON =====
claxon_pwm = PWM(Pin(CLAXON_PIN))
claxon_pwm.duty_u16(0)

# ===== FUNCIONES =====

def angle_to_duty(angle):
    min_duty = 1638   # ~0.5ms
    max_duty = 8192   # ~2.5ms
    return int(min_duty + (angle / 180) * (max_duty - min_duty))

def set_car(pwm_steering, s_angle, pwm_throttle, t_angle):
    pwm_steering.duty_u16(angle_to_duty(s_angle))
    pwm_throttle.duty_u16(angle_to_duty(t_angle))
    #print("Fuck")

def constrain(val, min_val, max_val):
    return max(min_val, min(max_val, val))

def process_command(line):
    # Limpiamos espacios y saltos de línea
    cmd = line.strip()
    #print(cmd)
    if not cmd:
        return

    # ===== CLAXON =====
    if cmd.startswith("B:"):
        if len(cmd) > 2 and cmd[2] == '1':
            claxon_pwm.freq(920)
            claxon_pwm.duty_u16(32768)
        else:
            claxon_pwm.duty_u16(0)
        return
    #print("Before try")
    # ===== PARSE STEERING/THROTTLE =====
    if "," in cmd:
        try:
            parts = cmd.split(',')
            #print(parts)
            s_val = constrain(float(parts[0]), 45, 135)
            t_val = constrain(float(parts[1]), 45, 135)
            #print("Valid command")
            #print(s_val, t_val)
            # Execution happens here
            set_car(steering_pwm, s_val, throttle_pwm, t_val)
        except (ValueError, IndexError) as ex:
            print(ex)
            pass

# ===== INIT =====
# Start at neutral
set_car(steering_pwm, 90, throttle_pwm, 90)
print("BLOCKING MODE: Waiting for command...")

# ===== MAIN BLOCKING LOOP =====
while True:
    # This line halts execution until a '\n' is received
    line = sys.stdin.readline()
    
    if line:
        process_command(line)
        # The loop only restarts after process_command finishes