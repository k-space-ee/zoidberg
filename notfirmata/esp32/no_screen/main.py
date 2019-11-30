from time import sleep_ms
from machine import Pin, PWM, Timer, I2C, reset, ADC

# How to use this
# 1. Flash MicroPython on the board
# 2. ampy -p /dev/ttyUSB0 put main.py
# 3. Reset the device
ESCON_MIN = 100
ESCON_WIDTH = 824
ESC_IDLE = 40


def toggle(pwm):
    pwm.deinit()
    sleep_ms(50)
    pwm.init()


timer_motors = Timer(1)
timer_sensor = Timer(2)

motors_enable = Pin(16, mode=Pin.OUT)
motors_enable.value(0)

motor1_speed = PWM(Pin(2, mode=Pin.OUT), freq=6000, duty=ESCON_MIN)
motor2_speed = PWM(Pin(12, mode=Pin.OUT), freq=6000, duty=ESCON_MIN)
motor3_speed = PWM(Pin(15, mode=Pin.OUT), freq=6000, duty=ESCON_MIN)
motor1_reverse = Pin(0, mode=Pin.OUT)
motor2_reverse = Pin(14, mode=Pin.OUT)
motor3_reverse = Pin(13, mode=Pin.OUT)

toggle(motor1_speed)
toggle(motor2_speed)
toggle(motor3_speed)

# relay
p = Pin(5, mode=Pin.OUT, value=0)
p.value(0)

# # ball sensor
pot = ADC(Pin(36))
pot.atten(ADC.ATTN_11DB)  # Full range: 3.3v

# ball loader
loader = PWM(Pin(4, mode=Pin.OUT), freq=50, duty=0)
toggle(loader)

for i in range(2):
    sleep_ms(500)
    loader.duty(40)
    sleep_ms(500)
    loader.duty(110)
    print("SWING!")


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


def restart():
    p.value(1)
    sleep_ms(1500)
    p.value(0)


def s():
    set_abc(1, 1, 1)


moving_average = []
pot_value = 999
counter = 0


def read_pot(t):
    global moving_average, pot_value, counter, loader
    counter += 1

    val = pot.read()
    moving_average = ([val] + moving_average)[:5]
    pot_value = int(sum(moving_average) / len(moving_average) / 10)
    if pot_value < 320:
        loader.duty(40)
    else:
        loader.duty(110)


timer_sensor.init(period=50, mode=Timer.PERIODIC, callback=read_pot)
d = timer_sensor.deinit
r = restart
l = loader.duty

motors_enable.value(1)
set_abce(0, 0, 0, 60)
restart()
