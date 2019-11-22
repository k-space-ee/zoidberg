import messenger
from distance_sense import CaptureDistance


class RealSenseNode(messenger.Node):

    def __init__(self, mock=False, run=True, silent=True) -> None:
        super().__init__('kicker_node', existing_loggers=['canbus'])
        self.publisher = messenger.Publisher('/distance/realsense', messenger.Messages.float)

        self.silent = silent
        self.mock = mock
        if not mock:
            self.sensor = CaptureDistance()

        if run:
            self.spin()

    def step(self):
        if self.mock:
            return

        self.publisher.publish(self.sensor.distance)
        self.loginfo_throttle(1, f"dist: {self.sensor.distance:.1f} area: {self.sensor.area:.1f}, fps: {self.sensor.fps:.0f}")


if __name__ == '__main__':
    node = RealSenseNode(mock=False, run=False)
    messenger.test()
    node.loop(30)
