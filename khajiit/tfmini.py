from time import sleep, time

import serial

import messenger
from serial_wrapper import *
from utils import StreamingMovingAverage


class TFMiniNode(messenger.Node):

    def __init__(self, run=True, mock=False) -> None:
        super().__init__('tfmini')
        self.publisher = messenger.Publisher('/distance/tfmini', messenger.Messages.float)

        result = self.get_serial()
        self.device = next(iter(result.keys()), None)
        print(repr(self.device), ": starting TF-mini")
        self.ser = None

        self.fps = self.distance = 0

        self.average = StreamingMovingAverage(3)
        self.average_fps = StreamingMovingAverage(20)

        while not self.open():
            sleep(2)

        if run:
            self.run()

    def get_serial(self):
        return find_serial("USB-Serial Controller") or find_serial("USB2.0-Serial")

    def open(self):
        try:
            result = self.get_serial()
            self.device = next(iter(result.keys()), None)
            if not self.device:
                return False
            self.ser = serial.Serial(self.device, 115200, timeout=1)
            if self.ser.is_open == False:
                self.ser.open()
            sleep(0.1)
            self.ser.in_waiting
            return True
        except:
            self.logger.error_throttle(1, "tf-mini can't open!")
            return False

    def run(self):

        try:
            start = time()
            counter = 0
            while True:
                counter += 1
                speak = counter % 100 == 0
                count = self.ser.in_waiting
                if count > 8:
                    recv = self.ser.read(9)
                    self.ser.reset_input_buffer()
                    low = recv[2]
                    high = recv[3]
                    distance = low + high * 256
                    if distance > 1000:
                        continue

                    # old = self.distance
                    # self.distance = self.average(distance)
                    self.distance = distance

                    self.publisher.publish(self.distance)
                    # if int(self.distance) != int(old):
                    if speak:
                        print(1, f"dist: {self.distance or 0:.1f}, fps: {self.fps:.0f}")
                    self.fps = self.average_fps(1 / (time() - start))
                    start = time()
                speak and print("tfmini")
                sleep(0.015)
        except OSError:
            self.logger.error("OSError!")
            self.ser.close()
        finally:
            self.ser.close()
        self.logcritical("tfMini DEAD")


if __name__ == '__main__':
    capture = TFMiniNode()
    capture.run()
