from time import sleep

import messenger
from realsense_distance import CaptureDistance


class RealSenseNode(messenger.Node):

    def __init__(self, mock=False, run=True, silent=True) -> None:
        super().__init__('realsense')
        self.publisher = messenger.Publisher('/distance/realsense', messenger.Messages.float)

        self.silent = silent
        self.mock = mock

        if not mock:
            self.sensor = CaptureDistance()

        if run:
            print("realsense looper")
            self.loop(30)

    def step(self):
        if self.mock:
            return

        self.publisher.publish(self.sensor.distance)
        self.loginfo_throttle(1, f"dist: {self.sensor.distance or 0:.1f} area: {self.sensor.area:.1f}, fps: {self.sensor.fps:.0f}")


if __name__ == '__main__':
    node = RealSenseNode(mock=False, run=False)
    messenger.test()
    node.loop(30)
