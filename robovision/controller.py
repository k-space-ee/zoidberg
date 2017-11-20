import serial
import logging

logger = logging.getLogger("esp32")


class Controller:
    def __init__(self, factor=0.2, maximum=0.02, path=None):
        logger.info("Opening /dev/ttyUSB0")
        self.ser = serial.Serial(port="/dev/ttyUSB0", baudrate=115200)
        self._has_ball = False
        self.factor = factor
        self.maximum = maximum

    def start(self):
        # pass
        self.ser.write(b"set_abce(0,0,0,50)\n\r")

    def set_abc(self, *speed):  # Ctrl-C doesn't work well,  Lauri tested b"\x03" +
        # print("before:", speed)
        if self.factor: speed = tuple(j * self.factor for j in speed)
        if self.maximum: speed = tuple(
            j * (min(self.maximum, abs(j)) / abs(j)) if j else j
            for j in speed
        )
        self.ser.write(("set_abc(%.4f,%.4f,%.4f)\n\r" % speed).encode("ascii"))

    def set_thrower(self, speed):
        self.ser.write(("set_thrower(%d)\n\r" % speed).encode("ascii"))

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
