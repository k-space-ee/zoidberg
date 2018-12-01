from machine import Pin, PWM, Timer, I2C, reset

# How to use this
# 1. Flash MicroPython on the board
# 2. ampy -p /dev/ttyUSB0 put main.py
# 3. Reset the device

# killer = Pin(4, mode=Pin.OUT)

timer_motors = Timer(1)
motors_enable = Pin(16, mode=Pin.OUT)
motors_enable.value(0)
motor1_speed = PWM(Pin(2, mode=Pin.OUT), freq=6000, duty=102)
motor2_speed = PWM(Pin(12, mode=Pin.OUT), freq=6000, duty=102)
motor3_speed = PWM(Pin(15, mode=Pin.OUT), freq=6000, duty=102)
motor1_reverse = Pin(0, mode=Pin.OUT)
motor2_reverse = Pin(14, mode=Pin.OUT)
motor3_reverse = Pin(13, mode=Pin.OUT)

ESCON_MIN = 100
ESCON_WIDTH = 824
ESC_IDLE = 40


def halt_motors(t):
    motor1_speed.duty(ESCON_MIN)
    motor2_speed.duty(ESCON_MIN)
    motor3_speed.duty(ESCON_MIN)


def set_abc(a, b, c):
    global ball_sensed
    ball_sensed = False
    assert -1 <= a <= 1, "M1 speed out of range -1.0 ... 1.0"
    assert -1 <= b <= 1, "M2 speed out of range -1.0 ... 1.0"
    assert -1 <= c <= 1, "M3 speed out of range -1.0 ... 1.0"
    motor1_speed.duty(ESCON_MIN + abs(int(a * ESCON_WIDTH)))
    motor2_speed.duty(ESCON_MIN + abs(int(b * ESCON_WIDTH)))
    motor3_speed.duty(ESCON_MIN + abs(int(c * ESCON_WIDTH)))
    motor1_reverse.value(a < 0)
    motor2_reverse.value(b < 0)
    motor3_reverse.value(c < 0)
    timer_motors.init(period=500, mode=Timer.ONE_SHOT, callback=halt_motors)


def set_abce(a, b, c, e):
    set_abc(a, b, c)


motors_enable.value(1)
set_abce(0, 0, 0, 60)
