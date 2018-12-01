import logging
import threading
from time import time, sleep
from typing import Optional

import yaml

import uavcan

from serial_wrapper import find_serial

logger = logging.getLogger("canbus")


class CanBusMotor:
    # TODO: this has the ability to not constantly update the speed, we should use that

    def __init__(self, kill=True) -> None:
        self.last_raw = ""
        self.last_msg = {}
        self.last_rpm = 0
        self.rpm = []
        self._speed = 0
        self.last_edit = time()
        self.kill = kill

        self.node: Optional[uavcan.node.Node] = None

        self.reconnect()

        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        while True:
            try:
                self.node.spin(timeout=1)  # Spin forever or until an exception is thrown
            except Exception as ex:
                logger.error('Node error: %s', ex)
                if self.node:
                    self.node.close()
                self.node = None
                self.reconnect()

    def reconnect(self):
        while True:
            zubax = set(find_serial('zubax').keys())
            if len(zubax) != 1:
                logger.error(f"Zubax controller not determined, {zubax}")
                sleep(1)
                continue

            serial_device = next(iter(zubax), None)
            logger.error(f"Zubax controller determined, {zubax}")
            self.node = node = uavcan.make_node(
                serial_device,
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

            # how fast can we blast this?
            node.periodic(0.05, self.update)
            node.add_handler(uavcan.equipment.esc.Status, self.listen)
            logger.info('New zubax node: %s', serial_device)

    @property
    def speed(self):
        return self._speed

    @speed.setter
    def speed(self, speed):
        speed = abs(speed)
        speed = min(15000, speed)
        self._speed = int(speed)
        self.last_edit = time()

    def update(self):
        if time() - self.last_edit > 0.8:
            if self._speed:
                print(self.last_raw)
            if self.kill:
                self._speed = 0
            else:
                self.last_edit = time()

        message = uavcan.equipment.esc.RPMCommand(rpm=[int(self._speed)])
        self.node.broadcast(message)

    def listen(self, msg):
        """
        Transfer(
            id=4, source_node_id=125, dest_node_id=None, transfer_priority=7,
            payload=uavcan.equipment.esc.Status(
                error_count=0, voltage=12.9296875,
                current=-0.0, temperature=307.0, rpm=0, power_rating_pct=0, esc_index=0)
            )
        """

        self.last_raw = uavcan.to_yaml(msg)
        self.last_msg = yaml.load(self.last_raw)
        self.rpm = (self.rpm + [self.last_msg.get('rpm', 0)])[-10:]
        self.last_rpm = round(sum(self.rpm) / len(self.rpm))
        self.last_msg['average_rpm'] = self.last_rpm


if __name__ == '__main__':
    kicker = CanBusMotor(kill=False)

    while True:
        try:
            speed = int(input("speed?: "))
            kicker.speed = speed
        except ValueError:
            pass

        print(kicker.last_raw)
