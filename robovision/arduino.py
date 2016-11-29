import os
import numpy as np
import serial
from threading import Thread, Event
from time import sleep, time
from math import cos, sin, radians, atan2, pi
import logging
logger = logging.getLogger("arduino")

class Arduino(Thread):
    def __init__(self, factor=0.4):
        Thread.__init__(self)
        self.daemon = True
        from configparser import ConfigParser
        cp = ConfigParser()
        cp.readfp(open(os.path.expanduser("~/.robovision/pymata.conf")))

        self.wake = Event() # wake up signal for the /dev observer
        self.changed = Event() # motor values have changed
        self.factor = factor

        indexes = 1, 2, 3
        device =  cp.get("global", "path")
        self.path = os.path.join("/dev/serial/by-path", device)
        self.kicker = cp.getint("global", "kicker")
        self.pwm = [cp.getint("motor%d" % j, "pwm") for j in indexes]
        self.en = [cp.getint("motor%d" % j, "en") for j in indexes]
        self.rev = [cp.getint("motor%d" % j, "rev") for j in indexes]

        # X, Y, rotation to A, B, C transformation matrix
        self.matrix = np.linalg.inv([
            [-1/2,     -1/2,      1],
            [3**0.5/2, -3**0.5/2, 0],
            [1,        1,         1]])
        self.alive = False
        self.running = False
        self.speed = None
        self.set_abc(0,0,0)
        self.board = None

        self.last_kick = time()

    def clean_up(self):
        if self.board:
            for pin in self.pwm:
                self.board.analog_write(pin, 255)
            for pin in self.en + self.rev:
                self.board.digital_write(pin, False)
            self.board.digital_write(self.kicker, True) # should close with discharge
        self.running = False

    def set_yw(self, y, w):
        """
        Set forward-backwards speed and rotation
        """
        self.set_abc(0.866*y, -0.866*y, w)

    def set_xyw(self, x, y, w):

        # x, y, w = [min(speed, 0.90) for speed in [x,y,w]]


        a = -0.5 * x + 0.866 * y + w
        b = -0.5 * x - 0.866 * y + w
        c = x + w

        m = max([abs(a), abs(b), abs(c)])
        if m > 1.0:
            a = a / m
            b = b / m
            c = c / m
        self.set_abc(a,b,c)

    def set_abc(self, *speed):
        for j in speed:
            assert j >= -1.0 and j <= 1.0
        speed = [j*self.factor for j in speed]
        if self.speed != speed:
            #logger.debug("Setting motor speeds: %s %s %s", *speed)
            self.speed = speed
            self.changed.set()

    def stop(self):
        self.running = False
        self.wake.set()
        self.changed.set()

    def run(self):
        try:
            from PyMata.pymata import PyMata
        except ImportError:
            logger.error("Pymata not installed, Arduino disabled!")
            return
        else:
            self.running = True
            while self.running:
                if not self.board:
                    if not os.path.exists(self.path):
                        logger.info("No Arduino, waiting for %s to become available", self.path)
                        self.wake.wait()
                    if not self.running:
                        break
                    board = PyMata(self.path, bluetooth=False)
                    try:
                        for pin in self.pwm:
                            logger.debug("Setting up pin %s as PWM output"%pin)
                            board.set_pin_mode(pin, board.PWM, board.DIGITAL)
                            board.analog_write(pin, 26)

                        for pin in self.rev:
                            logger.debug("Setting up pin %s as digital output" %pin)
                            board.set_pin_mode(pin, board.OUTPUT, board.DIGITAL)
                            board.digital_write(pin, 0)

                        for pin in self.en:
                            logger.debug("Setting up pin %s as digital output" %pin)
                            board.set_pin_mode(pin, board.OUTPUT, board.DIGITAL)
                            board.digital_write(pin, 1)

                        for pin in self.en:
                            board.set_pin_mode(pin, board.OUTPUT, board.DIGITAL)
                            board.digital_write(pin, 0)


                        board.set_pin_mode(self.kicker,   board.OUTPUT, board.DIGITAL)
                        board.digital_write(self.kicker, False)
                        board.set_pin_mode(13, board.INPUT, board.DIGITAL)
                        board.set_pin_mode(12, board.OUTPUT, board.DIGITAL)
                        board.digital_write(12, 0 )
                    except serial.serialutil.SerialException:
                        logger.error("Failed to connect to Arduino")
                        continue # Try again
                    else:
                        self.board = board
                        self.alive = True

                self.changed.wait()
                if not self.running:
                    break
                self.changed.clear()
                self.write()

                if self.last_kick + 0.1 < time():
                    self.set_kicker(False)

            self.set_abc(0,0,0)
            self.write()

    def write(self):
        try:
            for speed, pwm_pin, en_pin, rev_pin in zip(self.speed, self.pwm, self.en, self.rev):
                self.board.digital_write(en_pin, 1) #speed != 0)
                self.board.digital_write(rev_pin, speed < 0)
                speed = 24 + abs(int(float(speed) * 205))
                self.board.analog_write(pwm_pin, speed) # Set duty cycle
                # logger.info(str(speed))

        except serial.serialutil.SerialException:
            logger.info("Arduino disconnected at %s", self.path)
            self.alive = False
            self.board = None

    def set_kicker(self, value):
        self.board.digital_write(self.kicker, value)

    def kick(self):
        if self.last_kick + 0.5 < time():
            logger.info("{} whut".format(self.last_kick))
            self.set_kicker(True)
            self.last_kick = time()

if __name__ == "__main__":
    arduino = Arduino()
    arduino.start()

    sleep(5)
    """
    print("A")
    arduino.set_abc(-1,0,0)
    sleep(1)
    arduino.set_abc(1,0,0)
    sleep(1)

    print("B")
    arduino.set_abc(0,-1,0)
    sleep(1)
    arduino.set_abc(0,1,0)
    sleep(1)

    print("C")

    arduino.set_abc(0,0,-1)
    sleep(1)
    arduino.set_abc(0,0,1)
    sleep(1)
    """
    
    arduino.kick()
    sleep(2)

    arduino.stop()
    arduino.join()
