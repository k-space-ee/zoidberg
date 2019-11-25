from time import sleep_ms
from machine import Pin, PWM, Timer, I2C, reset
from micropython import const
import framebuf

# How to use this
# 1. Flash MicroPython on the board
# 2. ampy -p /dev/ttyUSB0 put main.py
# 3. Reset the device

# register definitions
SET_CONTRAST        = const(0x81)
SET_ENTIRE_ON       = const(0xa4)
SET_NORM_INV        = const(0xa6)
SET_DISP            = const(0xae)
SET_MEM_ADDR        = const(0x20)
SET_COL_ADDR        = const(0x21)
SET_PAGE_ADDR       = const(0x22)
SET_DISP_START_LINE = const(0x40)
SET_SEG_REMAP       = const(0xa0)
SET_MUX_RATIO       = const(0xa8)
SET_COM_OUT_DIR     = const(0xc0)
SET_DISP_OFFSET     = const(0xd3)
SET_COM_PIN_CFG     = const(0xda)
SET_DISP_CLK_DIV    = const(0xd5)
SET_PRECHARGE       = const(0xd9)
SET_VCOM_DESEL      = const(0xdb)
SET_CHARGE_PUMP     = const(0x8d)

class SSD1306:
    def __init__(self, width, height, external_vcc):
        self.width = width
        self.height = height
        self.external_vcc = external_vcc
        self.pages = self.height // 8
        self.buffer = bytearray(self.pages * self.width)
        fb = framebuf.FrameBuffer(self.buffer, self.width, self.height, framebuf.MONO_VLSB)
        self.framebuf = fb
        # Provide methods for accessing FrameBuffer graphics primitives. This is a
        # workround because inheritance from a native class is currently unsupported.
        # http://docs.micropython.org/en/latest/pyboard/library/framebuf.html
        self.fill = fb.fill
        self.pixel = fb.pixel
        self.hline = fb.hline
        self.vline = fb.vline
        self.line = fb.line
        self.rect = fb.rect
        self.fill_rect = fb.fill_rect
        self.text = fb.text
        self.scroll = fb.scroll
        self.blit = fb.blit
        self.init_display()

    def init_display(self):
        for cmd in (
            SET_DISP | 0x00, # off
            # address setting
            SET_MEM_ADDR, 0x00, # horizontal
            # resolution and layout
            SET_DISP_START_LINE | 0x00,
            SET_SEG_REMAP | 0x01, # column addr 127 mapped to SEG0
            SET_MUX_RATIO, self.height - 1,
            SET_COM_OUT_DIR | 0x08, # scan from COM[N] to COM0
            SET_DISP_OFFSET, 0x00,
            SET_COM_PIN_CFG, 0x02 if self.height == 32 else 0x12,
            # timing and driving scheme
            SET_DISP_CLK_DIV, 0x80,
            SET_PRECHARGE, 0x22 if self.external_vcc else 0xf1,
            SET_VCOM_DESEL, 0x30, # 0.83*Vcc
            # display
            SET_CONTRAST, 0xff, # maximum
            SET_ENTIRE_ON, # output follows RAM contents
            SET_NORM_INV, # not inverted
            # charge pump
            SET_CHARGE_PUMP, 0x10 if self.external_vcc else 0x14,
            SET_DISP | 0x01): # on
            self.write_cmd(cmd)
        self.fill(0)
        self.show()

    def poweroff(self):
        self.write_cmd(SET_DISP | 0x00)

    def poweron(self):
        self.write_cmd(SET_DISP | 0x01)

    def contrast(self, contrast):
        self.write_cmd(SET_CONTRAST)
        self.write_cmd(contrast)

    def invert(self, invert):
        self.write_cmd(SET_NORM_INV | (invert & 1))

    def show(self):
        x0 = 0
        x1 = self.width - 1
        if self.width == 64:
            # displays with width of 64 pixels are shifted by 32
            x0 += 32
            x1 += 32
        self.write_cmd(SET_COL_ADDR)
        self.write_cmd(x0)
        self.write_cmd(x1)
        self.write_cmd(SET_PAGE_ADDR)
        self.write_cmd(0)
        self.write_cmd(self.pages - 1)
        self.write_data(self.buffer)


class SSD1306_I2C(SSD1306):
    def __init__(self, width, height, i2c, addr=0x3c, external_vcc=False):
        self.i2c = i2c
        self.addr = addr
        self.temp = bytearray(2)
        super().__init__(width, height, external_vcc)

    def write_cmd(self, cmd):
        self.temp[0] = 0x80 # Co=1, D/C#=0
        self.temp[1] = cmd
        self.i2c.writeto(self.addr, self.temp)

    def write_data(self, buf):
        self.temp[0] = self.addr << 1
        self.temp[1] = 0x40 # Co=0, D/C#=1
        self.i2c.start()
        self.i2c.write(self.temp)
        self.i2c.write(buf)
        self.i2c.stop()



i2c = I2C(-1, Pin(4),Pin(5),freq=400000) # Bitbanged I2C bus
assert 60 in i2c.scan(), "No OLED display detected!"
oled = SSD1306_I2C(128, 64, i2c)
oled.invert(0) # White text on black background
oled.contrast(255) # Maximum contrast
oled.fill(0)
oled.text("booting.", 10, 10)
oled.show()

timer_motors = Timer(1)
motors_enable = Pin(16, mode=Pin.OUT)
motors_enable.value(0)
motor1_speed = PWM(Pin(2, mode=Pin.OUT), freq=6000, duty=102)
motor2_speed = PWM(Pin(12, mode=Pin.OUT), freq=6000, duty=102)
motor3_speed = PWM(Pin(15, mode=Pin.OUT), freq=6000, duty=102)
motor1_reverse = Pin(0, mode=Pin.OUT)
motor2_reverse = Pin(14, mode=Pin.OUT)
motor3_reverse = Pin(13, mode=Pin.OUT)


oled.text("booting..", 10, 10)
oled.show()


from machine import Pin, ADC
from time import sleep

pot = ADC(Pin(36))
pot.atten(ADC.ATTN_11DB)       #Full range: 3.3v

moving_average = []
while True:
  pot_value = pot.read()
  moving_average = ([pot_value] + moving_average)[:20]
  pot_value = int(sum(moving_average) / len(moving_average) / 10)
  print(pot_value, pot_value < 14)
  sleep(0.01)


from time import sleep_ms
from machine import Pin, PWM, Timer, I2C, reset
esc = PWM(Pin(4, mode=Pin.OUT), freq=50, duty=0);esc.duty(70);
while True:
    sleep_ms(1000)
    esc.duty(40)
    sleep_ms(1000)
    esc.duty(110)


# Turn off ESC pin
timer_thrower = Timer(2)
esc = PWM(Pin(265, mode=Pin.OUT), freq=50, duty=0);esc.duty(70);
sleep_ms(2000)
oled.text("booting...", 10, 10)
oled.show()

# Initiliaze ESC
esc.duty(40);
sleep_ms(2000)
oled.text("booting....", 10, 10)
oled.show()

ESCON_MIN = 100
ESCON_WIDTH = 824
ESC_IDLE = 40

def redraw(t):
    oled.fill(0)
    for i, m in enumerate([motor1_speed, motor2_speed, motor3_speed]):
        d = m.duty()
        if d == ESCON_MIN:
            oled.text("M%d  halted" % (i+1), 10, i*10)
        else:
            oled.text("M%d:" % (i+1), 10, i*10)
            oled.rect(39, i*10, int(ESCON_WIDTH/10)+2, 9, 1)
            oled.fill_rect(40, i*10+1, int((d-ESCON_MIN)/10), 7, 1)

    d = esc.duty()
    if d == ESC_IDLE:
      oled.text("ESC halted", 10, 40)
    else:
      oled.text("ESC: %d" % d, 10, 40)
    oled.show()

def halt_motors(t):
    motor1_speed.duty(ESCON_MIN)
    motor2_speed.duty(ESCON_MIN)
    motor3_speed.duty(ESCON_MIN)

def halt_thrower(t):
    esc.duty(ESC_IDLE)

def set_abc(a,b,c):
    global ball_sensed
    ball_sensed = False
    assert -1 <= a <= 1,  "M1 speed out of range -1.0 ... 1.0"
    assert -1 <= b <= 1,  "M2 speed out of range -1.0 ... 1.0"
    assert -1 <= c <= 1,  "M3 speed out of range -1.0 ... 1.0"
    motor1_speed.duty(ESCON_MIN + abs(int(a * ESCON_WIDTH)))
    motor2_speed.duty(ESCON_MIN + abs(int(b * ESCON_WIDTH)))
    motor3_speed.duty(ESCON_MIN + abs(int(c * ESCON_WIDTH)))
    motor1_reverse.value(a < 0)
    motor2_reverse.value(b < 0)
    motor3_reverse.value(c < 0)
    timer_motors.init(period=500, mode=Timer.ONE_SHOT, callback=halt_motors)

def set_thrower(e):
    global ball_sensed
    ball_sensed = False
    assert 40 <= e <= 110, "ESC speed out of range"
    if e == ESC_IDLE:
        esc.duty(ESC_IDLE)
    else:
        esc.duty(e +  int((12 - battery_voltage()) / 0.2))
    timer_thrower.init(period=3000, mode=Timer.ONE_SHOT, callback=halt_thrower)

def set_abce(a,b,c,e):
    set_abc(a,b,c)
    set_thrower(e)

def a(): # approach ball
    for j in range(0,20):
        set_abce(0.05,-0.05,0,80)
        sleep_ms(100)

def f(): # forward
    set_abce(0.1,-0.1,0,ESC_IDLE)

def b():
    set_abce(-0.1,0.1,0,ESC_IDLE)

def tr(): # turn right
    set_abce(-0.1,-0.1,-0.1,ESC_IDLE)

def tl(): # turn left
    set_abce(0.1,0.1,0.1,ESC_IDLE)

def sr(): # strafe right
    set_abce(-0.1,-0.1,0.86,ESC_IDLE)

def sl(): # strafe left
    set_abce(0.1,0.1,-0.86,ESC_IDLE)


from machine import reset
grabbing = False
ball_sensed = False
timer_suck = Timer(4)
grabbing_speed = None

def got_callback(p):
    global grabbing
    global ball_sensed
    timer_suck.deinit()
    grabbing = False
    ball_sensed = True
    set_abc(0,0,0)
    esc.duty(ESC_IDLE) # DONT READ BATTERY VOLTAGE!!!

pin_sensor = Pin(39, Pin.IN)
pin_sensor.irq(trigger=Pin.IRQ_FALLING, handler=got_callback)

grabbing_pulse = 200
grabbing_idle = 130

def stop(timer=None):
    esc.duty(ESC_IDLE) # DONT READ BATTERY VOLTAGE!!!
    if grabbing and pin_sensor.value():
        set_abc(grabbing_speed, 0, -grabbing_speed)
        timer_suck.init(period=grabbing_pulse, mode=Timer.ONE_SHOT, callback=suck)
    
def suck(timer=None):
    if grabbing and pin_sensor.value():
        set_abc(grabbing_speed, 0, -grabbing_speed)
        esc.duty(100)
        timer_suck.init(period=grabbing_idle, mode=Timer.ONE_SHOT, callback=stop)
    else:
        esc.duty(ESC_IDLE) # DONT READ BATTERY VOLTAGE!!!

# grab(200, 130, 0.01)
def grab(pulse=200, idle=130, speed=0.01):
    global grabbing
    global ball_sensed
    global grabbing_speed
    global grabbing_pulse
    global grabbing_idle
    grabbing_pulse = pulse
    grabbing_idle = idle
    grabbing_speed = speed
    if grabbing and ball_sensed: # we have ball
        return "ball_sensed"
    elif not grabbing:
        grabbing = True
        suck()
        return "not_sensed"
    else:
        return "already_grabbing"



from machine import Pin, ADC
adc = ADC(Pin(36, Pin.IN))
adc.atten(ADC.ATTN_0DB)


def battery_voltage():
    for j in range(0,2): # skip garbage
        adc.read()
    s = 0
    for j in range(0,3):
        s += adc.read()
    return s * 0.0041 / (j+1)





motors_enable.value(1)
set_abce(0,0,0,60)
oled.text("booting.....", 10, 10)
oled.show()


timer_redraw = Timer(3)
timer_redraw.init(period=100, mode=Timer.PERIODIC, callback=redraw)

