from time import sleep
from ucollections import OrderedDict
from machine import Pin, PWM, Timer, I2C, reset
from micropython import const
import framebuf
import ujson

# How to use this
# 1. Flash MicroPython on the board
# 2. ampy -p /dev/ttyUSB0 put main.py
# 3. Reset the device

# register definitions
SET_CONTRAST = const(0x81)
SET_ENTIRE_ON = const(0xa4)
SET_NORM_INV = const(0xa6)
SET_DISP = const(0xae)
SET_MEM_ADDR = const(0x20)
SET_COL_ADDR = const(0x21)
SET_PAGE_ADDR = const(0x22)
SET_DISP_START_LINE = const(0x40)
SET_SEG_REMAP = const(0xa0)
SET_MUX_RATIO = const(0xa8)
SET_COM_OUT_DIR = const(0xc0)
SET_DISP_OFFSET = const(0xd3)
SET_COM_PIN_CFG = const(0xda)
SET_DISP_CLK_DIV = const(0xd5)
SET_PRECHARGE = const(0xd9)
SET_VCOM_DESEL = const(0xdb)
SET_CHARGE_PUMP = const(0x8d)


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
                SET_DISP | 0x00,  # off
                # address setting
                SET_MEM_ADDR, 0x00,  # horizontal
                # resolution and layout
                SET_DISP_START_LINE | 0x00,
                SET_SEG_REMAP | 0x01,  # column addr 127 mapped to SEG0
                SET_MUX_RATIO, self.height - 1,
                SET_COM_OUT_DIR | 0x08,  # scan from COM[N] to COM0
                SET_DISP_OFFSET, 0x00,
                SET_COM_PIN_CFG, 0x02 if self.height == 32 else 0x12,
                # timing and driving scheme
                SET_DISP_CLK_DIV, 0x80,
                SET_PRECHARGE, 0x22 if self.external_vcc else 0xf1,
                SET_VCOM_DESEL, 0x30,  # 0.83*Vcc
                # display
                SET_CONTRAST, 0xff,  # maximum
                SET_ENTIRE_ON,  # output follows RAM contents
                SET_NORM_INV,  # not inverted
                # charge pump
                SET_CHARGE_PUMP, 0x10 if self.external_vcc else 0x14,
                SET_DISP | 0x01):  # on
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
        self.temp[0] = 0x80  # Co=1, D/C#=0
        self.temp[1] = cmd
        self.i2c.writeto(self.addr, self.temp)

    def write_data(self, buf):
        self.temp[0] = self.addr << 1
        self.temp[1] = 0x40  # Co=0, D/C#=1
        self.i2c.start()
        self.i2c.write(self.temp)
        self.i2c.write(buf)
        self.i2c.stop()


i2c = I2C(-1, Pin(4), Pin(5), freq=400000)  # Bitbanged I2C bus
assert 60 in i2c.scan(), "No OLED display detected!"
oled = SSD1306_I2C(128, 64, i2c)
oled.invert(0)  # White text on black background
oled.contrast(255)  # Maximum contrast
oled.fill(0)
oled.show()

##############
flip = 0
data = []
logo = (
   "\    ||||__   o",
   "| \_/    o \ o",
   "> _   (( <_ o",
   "| / \__+___/",
   "|/     |/",
   "",
   "Sick fish hackin",
)
placeholder_data = [logo]
config = {
    'field_id': 'B',
    'gameplay status': 'disabled',
    'robot_id': 'A',
    'target goal color': 'purple',
}

config_json = ujson.dumps(config)
config_package = "!@#$%s!@#$" % config_json

field_jumper = Pin(15, Pin.IN, Pin.PULL_UP)
goal_jumper = Pin(13, Pin.IN, Pin.PULL_UP)
robot_jumper = Pin(12, Pin.IN, Pin.PULL_UP)


def refresh_config():
    global config, config_json, config_package

    config = {
        'field_id': 'A' if field_jumper.value() else 'B',
        'gameplay status': 'disabled',
        'robot_id': 'A' if robot_jumper.value() else 'B',
        'target goal color': 'blue' if goal_jumper.value() else 'purple',
    }

    config_json = ujson.dumps(config)
    config_package = "!@#$%s!@#$" % config_json


def render(*_, extra=None):
    global flip, placeholder_data

    to_show = data or placeholder_data
    flip = (flip + 1) % len(to_show)

    refresh_config()

    oled.fill(0)

    for i, row in enumerate(to_show[flip]):
        oled.text(row, 0, 8 * i)

    if extra:
        oled.text(extra, 0, 8 * (i + 2))

    oled.show()

    placeholder_data = [
        tuple(
            '{:<8}: {}'.format(k[:8], v) for k, v in config.items()
        )
    ]


render()

timer_redraw = Timer(1)
timer_redraw.init(period=2000, mode=Timer.PERIODIC, callback=render)


# if no machine input, kill oled screen, as it blocks flashing
def kill(*t):
    global data
    if not data:
        timer_redraw.deinit()
        render(extra="FLASHING ALLOW")


timer_kill = Timer(2)
timer_kill.init(period=10000, mode=Timer.ONE_SHOT, callback=kill)
