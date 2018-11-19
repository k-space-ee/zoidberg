import messenger
from kicker import CanBusMotor


class KickerNode(messenger.Node):

    def __init__(self, mock=False, run=True) -> None:
        super().__init__('kicker_node')
        self.reader = messenger.Reader('/kicker_speed', messenger.Messages.integer, callback=self.callback)
        self.publisher = messenger.Publisher('/canbus_message', messenger.Messages.string)

        self.mock = mock
        if not mock:
            self.controller = CanBusMotor()

        if run:
            self.spin()

    def callback(self):
        speed = self.reader.last_reading

        if not self.mock:
            self.controller.speed = speed
            self.publisher.publish(self.controller.last_raw)

        self.logger(
            ["speed", speed],
        )


if __name__ == '__main__':
    node = KickerNode(mock=True, run=False)
    messenger.test()
    node.spin()
