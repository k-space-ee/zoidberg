from threading import Thread
from enum import Enum
import serial
from time import sleep
from config_manager import ConfigManager


game_config = ConfigManager("game")
class State(Enum):
    start = 1
    field_id = 2
    robot_id = 3
    command = 4
    command_all = 5
    end = 6


class RemoteRF(Thread):
    def __init__(self,dev,gameplay=None):
        Thread.__init__(self)
        self.ser = serial.Serial(dev, timeout=1)
        self.daemon = True
        self._stop = False
        self._fight = False
        self.gameplay = gameplay


    @property
    def fight(self):
        """Return True or False based on judge remotecontrol"""
        return self._fight

    def parse_packet(self, packet):
        pass

    def stop(self):
        self._stop = True

    def run(self):
        print("REMOTERF THREAD STARTED")
        state = State.start
        string = ""

        while 1:
            ack_packet = "a{}{}ACK------".format(
                game_config.get_value("global", "field_id"),
                game_config.get_value("global", "robot_id"),
                )
            ack_packet = ack_packet.encode()
            c = self.ser.read(11)

            field, robot = game_config.get_value("global", "field_id"), game_config.get_value("global", "robot_id")

            try:
                c = c.decode()
            except:
                continue
            if not c:
                continue
            string += c

            if "START" in string:
                print("GOT START")
                string = ""
                if self.gameplay and not self.gameplay.alive:
                    self.gameplay.enable()

            if "STOP" in string:
                print("GOT STOP")
                string = ""
                if self.gameplay and self.gameplay.alive:
                    self.gameplay.disable()

            if "PING" in string:
                print("got ping")
                string = ""
                self.ser.write(ack_packet)

            if len(string) > 1000:
                string = ""
            
if __name__ == "__main__":
    rf = RemoteRF("/dev/serial/by-path/pci-0000:00:14.0-usb-0:2.1:1.0-port0")
    rf.start()
    while True:
        sleep(0.1)
    rf.stop()
