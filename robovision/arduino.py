import os
import numpy as np
import serial
from threading import Thread, Event
from time import sleep, time
from math import cos, sin, radians, atan2, pi
import logging
import itertools
import struct
logger = logging.getLogger("arduino")

class Arduino(Thread):
    def __init__(self, factor=0.4, path=None):
        Thread.__init__(self)
        self.daemon = True
        self.wake = Event() # wake up signal for the /dev observer
        self.changed = Event() # motor values have changed
        self.factor = factor

        if not path:
            #indexes = 1, 2, 3
            from configparser import ConfigParser
            cp = ConfigParser()
            cp.readfp(open(os.path.expanduser("~/.robovision/pymata.conf")))
            device =  cp.get("global", "path")
            self.path = os.path.join("/dev/serial/by-path", device)
            #self.kicker = cp.getint("global", "kicker")
            #self.pwm = [cp.getint("motor%d" % j, "pwm") for j in indexes]
            #self.en = [cp.getint("motor%d" % j, "en") for j in indexes]
            #self.rev = [cp.getint("motor%d" % j, "rev") for j in indexes]
        self.path = path

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
        self._packet = []
        self._checksum = 0
        self.last_kick = time()
        self.grabber = False
        self.kicker = False

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

    def parse(self, data):
        """
        Parse a packet from Arduino
        """
        #print(len(self._packet), hex(data))
        rlen = len(self._packet)
        if data == 0xAA and rlen == 0:
            self._packet.append(data)
            #print("First")
            return
        if data == 0xAA and rlen == 1:
            self._packet.append(data)
            #print("Second")
            return

        if rlen > 1 and rlen < 7:
            self._packet.append(data)
            self._checksum += data & 0xff
            if self._checksum > 255:
                self._checksum &= 0x00FF
            return
        elif rlen == 7:
            self._packet.append(data)
            if self._checksum == data:
                #print("<", ' '.join('{:02x}'.format(x) for x in self._packet))
                self._checksum = 0
                self.alive = True
                del self._packet[:]
                return
            else:
                print("SERR <", ' '.join('{:02x}'.format(x) for x in self._packet))
                self._checksum = 0
                del self._packet[:]
                return
        else:
            logger.info("Arduino packet error")
            self._checksum = 0
            del self._packet[:]
            return


    def write(self):
        packet = [0xaa, 0xaa]
        enables = []
        directions = []
        for speed in self.speed:
            enables.append(True)
            directions.append(speed > 0)
            pwm = 24 + abs(int(float(speed) * 205))
            packet.append(pwm & 0xff)
        bitlist = itertools.chain((self.kicker, self.grabber), directions[::-1], enables[::-1])
        bits = "".join(["1" if i else "0" for i in bitlist][::1])
        bits = int(bits, 2)
        packet.append(bits)

        #print(packet)
        checksum = 0
        for byte in packet[2:]:
            checksum += byte
        packet.append(checksum & 0xff)
        #print(">", ' '.join('{:02x}'.format(x) for x in packet))
        final = struct.pack("7B", *packet)
        #sprint(" ".join("{:02x}".format(c) for c in final))
        try:
            self.board.write(final)
            self.board.flush()
        except serial.serialutil.SerialException:
            logger.info("Arduino disconnected at %s", self.path)
            self.alive = False
            self.board = None


    def restart_arduino(self, board=None):
        if self.board:
            board = self.board
        if board:
            logger.debug("Restaring arduino over ttyUSB")
            board.setDTR(False)
            sleep(0.022)
            board.flushInput()
            board.setDTR(True)

    def run(self):
        self.running = True
        while self.running:
            if not self.board:
                if not os.path.exists(self.path):
                    logger.info("No Arduino, waiting for %s to become available", self.path)
                    self.wake.wait()
                if not self.running:
                    break
                logger.info("Trying to connect to a Arduino at "+self.path)
                board = serial.Serial(self.path, 9600,
                                    bytesize=serial.EIGHTBITS,
                                    parity=serial.PARITY_NONE,
                                    stopbits=serial.STOPBITS_ONE,
                                    timeout=1,
                                    xonxoff=0,
                                    rtscts=0)
                try:
                    logger.info("Waing for arduino to start talking to me")
                    self.restart_arduino(board)
                    while not board.inWaiting():
                        logger.debug("Waiting for arduino")
                        sleep(0.1)
                    logger.info("Arduino is talking")

                except serial.serialutil.SerialException:
                    logger.error("Failed to connect to Arduino")
                    continue # Try again
                else:
                    self.board = board
                    self.grabber = True

            if self.board.inWaiting() > 0:
                data = self.board.read(self.board.inWaiting())
                for byte in data:
                    self.parse(byte)


            if not self.running:
                break
            self.changed.clear()
            self.write()

            if self.last_kick + 0.1 < time():
                self.set_kicker(False)

        self.set_abc(0,0,0)
        self.write()

    """
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
    """

    def set_kicker(self, value):
        self.kicker = value

    def kick(self):
        self.set_kicker(True)
        logger.info("KICK {}, {}".format(time(), time()-self.last_kick))
        self.last_kick = time()

if __name__ == "__main__":
    logger.addHandler(logging.StreamHandler())
    logger.setLevel(logging.DEBUG)
    arduino = Arduino(path="/dev/ttyUSB0")
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
