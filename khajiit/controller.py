import logging
from time import sleep

import serial

from serial_wrapper import find_serial

logger = logging.getLogger("esp32")


class Controller:
    # Ctrl-C doesn't work well,  Lauri tested b"\x03" +
    # safe values 0.2 0.05
    def __init__(self, factor=0.2, maximum=0.065):
        self.motor_serial = None

        self.factor = factor
        self.maximum = maximum

        self.state = [0, 0, 0, 0]

    def assert_config(self):
        speeds, shooter = self.state[:-1], self.state[-1]

        if max(map(abs, speeds)) > 1:
            logger.error("speeds out of range %s", str(speeds), max(map(abs, speeds)))
            return False

        if shooter < 0 or shooter > 15000:
            logger.error("shooter speed out of range %f", shooter)
            return False

        return True

    def try_reconnect(self):
        if self.motor_serial:
            self.motor_serial.close()
        self.motor_serial = None
        for i in range(10):
            if self.motor_serial is not None:
                break
            controller_serial = next(iter(find_serial('CP2102')), None)
            logger.info("Opening %s", controller_serial)

            if not controller_serial:
                logger.error('Reconnect serial device not found!')
                sleep(1)
                continue

            try:
                self.motor_serial = serial.Serial(
                    port=controller_serial,
                    baudrate=115200, xonxoff=True, timeout=0.01)
            except Exception as e:
                logger.error('Reconnect failed: %s', e)
                sleep(1)

    def command(self, command):
        motor_serial = self.motor_serial
        try:
            motor_serial.write(command.encode("ascii"))
        except Exception as e:
            logger.error("Command failed: %s", e)
            self.try_reconnect()

    def apply(self):
        # check if our state is valid
        if not self.assert_config():
            return

        motor_serial = self.motor_serial
        speed = [str(round(s, 5)) for s in self.state[:-1]]
        a, c, b = speed

        # send a python command to esp32
        command = "set_abce(%s,%s,%s,%d)\n\r" % (a, b, c, 40)

        try:
            motor_serial.write(command.encode("ascii"))
        except Exception as e:
            logger.error("Motors dead: %s", e)
            self.try_reconnect()

    def set_abc(self, *speed):
        if self.factor:
            speed = tuple(j * self.factor for j in speed)

        if self.maximum:
            speed = tuple(
                j * (min(self.maximum, abs(j)) / abs(j)) if j else j
                for j in speed
            )
        self.state = list(speed) + self.state[-1:]

    def set_thrower(self, speed):
        self.state[-1] = speed

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
