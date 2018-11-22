import messenger
from controller import Controller


class ControllerNode(messenger.Node):

    def __init__(self, mock=False, run=True, **kwargs) -> None:
        super().__init__('motion_node', existing_loggers=['esp32'], **kwargs)
        self.listener = messenger.Listener('/movement', messenger.Messages.motion, callback=self.callback)
        self.mock = mock
        if not mock:
            self.controller = Controller()

        self.logger.info("Start")
        if run:
            self.spin()

    def callback(self, last_reading):
        linear = last_reading.linear
        angular = last_reading.angular

        x, y, z = linear.x, linear.y, linear.z
        ax, ay, az = angular.x, angular.y, angular.z

        if not self.mock:
            self.controller.set_xyw(x, y, az)
            self.controller.apply()

        self.loginfo_throttle(1, "speeds %.2f %.2f %.2f" % (x, y, az))


if __name__ == '__main__':
    node = ControllerNode(mock=True, run=False)
    messenger.test()
    node.spin()
