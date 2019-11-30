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
            self.iter = iter(self.sensor)

        if run:
            print("realsense looper")
            self.loop(35)

    def step(self):
        if self.mock:
            return

        is_running = next(self.iter, None)
        if not is_running:
            exit()

        self.publisher.publish(self.sensor.distance)
        delay = 2.5 if self.sensor.distance else 4
        self.loginfo_throttle(delay, f"dist: {self.sensor.distance or 0:.1f} area: {self.sensor.area:.1f}, fps: {self.sensor.fps:.0f}")


if __name__ == '__main__':
    node = RealSenseNode(mock=False, run=False)
    messenger.test()
    node.loop(30)
