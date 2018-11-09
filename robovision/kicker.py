import threading
import uavcan, time
from time import time


# TODO: this has the ability to not constantly update the speed, we should use that

class CanBusMotor:

    def __init__(self) -> None:
        super().__init__()
     
        self.last_message = ""
        self._speed = 0
        self.last_edit = time()

        self.node = node = uavcan.make_node(
            '/dev/serial/by-id/usb-Zubax_Robotics_Zubax_Babel_32002E0018514D563935392000000000-if00',
            node_id=10,
            bitrate=1000000,
        )

        # setup
        node_monitor = uavcan.app.node_monitor.NodeMonitor(node)
        dynamic_node_id_allocator = uavcan.app.dynamic_node_id.CentralizedServer(node, node_monitor)

        # Waiting for at least one other node to appear online (our local node is already online).
        while len(dynamic_node_id_allocator.get_allocation_table()) <= 1:
            print('Waiting for other nodes to become online...')
            node.spin(timeout=1)

        node.periodic(0.05, self.update)
        node.add_handler(uavcan.equipment.esc.Status, self.listen)

        self.thread = threading.Thread(target=node.spin, daemon=True).start()

    @property
    def speed(self):
        return self.__speed

    @speed.setter
    def speed(self, speed):
        self._speed = speed
        self.last_edit = time()

    def update(self):
        if time() - self.last_edit > 2:
            self._speed = 0

        message = uavcan.equipment.esc.RawCommand(cmd=[self._speed])
        self.node.broadcast(message)

    def listen(self, msg):
        self.last_message = uavcan.to_yaml(msg)


if __name__ == '__main__':

    kicker = CanBusMotor()

    while True:
        try:
            speed = int(input("speed?: "))
            kicker.speed = speed
        except ValueError:
            pass

        print(kicker.last_message)
