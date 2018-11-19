import messenger
from controller import Controller


class ControllerNode(messenger.Node):

    def __init__(self, mock=False, run=True) -> None:
        super().__init__('motion_node')
        self.reader = messenger.Reader('/movement', messenger.Messages.motion, callback=self.callback)
        self.mock = mock
        if not mock:
            self.controller = Controller()

        if run:
            self.spin()

    def callback(self):
        last_reading = self.reader.last_reading

        linear = last_reading.linear
        angular = last_reading.angular

        x, y, z = linear.x, linear.y, linear.z
        ax, ay, az = angular.x, angular.y, angular.z

        if not self.mock:
            self.controller.set_xyw(y, x, az)
            self.controller.apply()

        self.logger(
            ["linear", [x, y, z], "angular", [ax, ay, az]],
        )


if __name__ == '__main__':
    node = ControllerNode(mock=True, run=False)
    messenger.test()
    node.spin()
