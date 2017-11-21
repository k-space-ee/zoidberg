from time import time

import serial
import logging

logger = logging.getLogger("esp32")


class Controller:
    def __init__(self, factor=0.2, maximum=0.02, path=None):
        logger.info("Opening /dev/ttyUSB0")
        self.ser = serial.Serial(
            port="/dev/serial/by-path/pci-0000:00:14.0-usb-0:1.4:1.0-port0",
            baudrate=115200, xonxoff=True)
        self._has_ball = False
        self.factor = factor
        self.maximum = maximum
        self.state = [0,0,0,40]
        self.time = time()

    def start(self):
        self.ser.write(b"set_abce(0,0,0,70)\n\r")

    def apply(self):
        s = self.ser
        speed = [str(round(s,3)) for s in self.state[:-1]] + self.state[-1:]
        s.write(("set_abce(%s,%s,%s,%d)\n\r" % tuple(speed)).encode("ascii"))

    def set_abc(self, *speed):  # Ctrl-C doesn't work well,  Lauri tested b"\x03" +
        # print("before:", speed)
        if self.factor: speed = tuple(j * self.factor for j in speed)
        if self.maximum: speed = tuple(
            j * (min(self.maximum, abs(j)) / abs(j)) if j else j
            for j in speed
        )
        self.state = list(speed) + self.state[-1:]

    def set_thrower(self, speed):
        self.state = self.state[:-1] + [speed]

    def set_yw(self, y, w):
        """
        Set forward-backwards speed and rotation
        """
        self.set_abc(0.866 * y, -0.866 * y, w)

    def set_xyw(self, x, y, w):
        a = -0.5 * x + 0.866 * y + w
        b = -0.5 * x - 0.866 * y + w
        c = x + w

        m = max([abs(a), abs(b), abs(c)])
        if m > 1.0:
            a = a / m
            b = b / m
            c = c / m

        self.set_abc(a, b, c)

    def stop(self):
        self.running = False

    def kick(self):
        pass

    @property
    def has_ball(self):
        return self._has_ball

    def set_grabber(self, value):
        self.grabber = value
