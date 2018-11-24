import messenger
from kicker import CanBusMotor


class KickerNode(messenger.Node):

    def __init__(self, mock=False, run=True) -> None:
        super().__init__('kicker_node')
        self.listener = messenger.Listener('/kicker_speed', messenger.Messages.integer, callback=self.callback)
        self.publisher = messenger.Publisher('/canbus_message', messenger.Messages.string)

        self.mock = mock
        if not mock:
            self.controller = CanBusMotor()

        if run:
            self.spin()

    def callback(self, speed):
        last_rpm = None
        if not self.mock:
            self.controller.speed = speed.data
            if self.controller.last_msg:
                self.publisher.command(**self.controller.last_msg)
                last_rpm = self.controller.last_rpm

        self.loginfo_throttle(1, f"set rpm {speed}, current: {last_rpm}")


if __name__ == '__main__':
    node = KickerNode(mock=True, run=False)
    messenger.test()
    node.spin()
