from time import sleep, time

import serial

import messenger
from serial_wrapper import *
from utils import StreamingMovingAverage


class TFMiniNode(messenger.Node):

    def __init__(self, run=True, mock=False) -> None:
        super().__init__('tfmini')
        self.publisher = messenger.Publisher('/distance/tfmini', messenger.Messages.float)

        result = find_serial("USB-Serial Controller")
        self.device = list(result.keys()).pop()
        print(repr(self.device), ": starting TF-mini")
        self.ser = None

        self.fps = self.distance = 0

        self.average = StreamingMovingAverage(5)
        self.average_fps = StreamingMovingAverage(20)
        self.ser = serial.Serial(self.device, 115200, timeout=1)
        if self.ser.is_open == False:
            self.ser.open()

        if run:
            self.run()

    def run(self):
        try:
            start = time()
            while True:
                count = self.ser.in_waiting
                if count > 8:
                    recv = self.ser.read(9)
                    self.ser.reset_input_buffer()
                    low = recv[2]
                    high = recv[3]
                    distance = low + high * 256
                    if distance > 1000:
                        continue
                        
                    self.distance = self.average(distance)

                    self.publisher.publish(self.distance)
                    self.loginfo_throttle(2, f"dist: {self.distance or 0:.1f}, fps: {self.fps:.0f}")
                    self.fps = self.average_fps(1 / (time() - start))
                    start = time()

                sleep(0.001)
        finally:
            self.ser.close()


if __name__ == '__main__':
    capture = TFMiniNode()
    capture.run()
