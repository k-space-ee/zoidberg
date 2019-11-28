from machine import Pin, PWM, Timer, I2C, reset, ADC
from time import sleep_ms

# How to use this
# 1. Flash MicroPython on the board
# 2. ampy -p /dev/ttyUSB0 put main.py
# 3. Reset the device

test_timer = Timer(1)

# relay
p = Pin(5, mode=Pin.OUT)

# ball sensor
pot = ADC(Pin(36))
pot.atten(ADC.ATTN_11DB)  # Full range: 3.3v

# ball loader
loader = PWM(Pin(4, mode=Pin.OUT), freq=50, duty=0)

# motors
motors_enable = Pin(16, mode=Pin.OUT)

motor1_speed = PWM(Pin(2, mode=Pin.OUT), freq=50, duty=0)
motor2_speed = PWM(Pin(12, mode=Pin.OUT), freq=50, duty=0)
motor3_speed = PWM(Pin(15, mode=Pin.OUT), freq=50, duty=0)

motor1_reverse = Pin(0, mode=Pin.OUT)
motor2_reverse = Pin(14, mode=Pin.OUT)
motor3_reverse = Pin(13, mode=Pin.OUT)

moving_average = []
pot_value = -999


def run_test(t):
    global moving_average, pot_value, counter
    counter += 1

    pot_value = pot.read()

    out = int(counter % 2)
    pwm = 40 + 70 * out

    p.value(out)
    motors_enable.value(out)
    motor1_reverse.value(out)
    motor2_reverse.value(out)
    motor3_reverse.value(out)

    loader.duty(pwm)
    motor1_speed.duty(pwm)
    motor2_speed.duty(pwm)
    motor3_speed.duty(pwm)

    print(out, pwm, pot_value)


test_timer.init(period=1000, mode=Timer.PERIODIC, callback=run_test)
d = test_timer.deinit
