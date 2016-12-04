from threading import Thread
from enum import Enum
import serial
from time import sleep

class State(Enum):
    start = 1
    field_id = 2
    robot_id = 3
    command = 4
    command_all = 5
    end = 6


class RemoteRF(Thread):
    def __init__(self, gameplay, dev):
        Thread.__init__(self)
        self.ser = serial.Serial(dev, timeout=1)
        self.field_id = gameplay.field_id
        self.robot_id = gameplay.robot_id
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
        state = State.start
        while not self._stop:
            ack_packet = "a{}{}ACK------".format(self.field_id, self.robot_id)
            ack_packet = ack_packet.encode()
            c = self.ser.read(1)
            try:
                c = c.decode()
            except:
                continue
            if not c:
                continue
            if state == State.start and c == 'a':
                state = State.field_id
                continue
            if state == State.field_id:
                if c == self.field_id:
                    state = State.robot_id
                    continue
                else:
                    state = State.start
                    continue
            if state == State.robot_id:
                if c == self.robot_id:
                    state = State.command
                    continue
                elif c == 'X':
                    state = State.command_all
                    continue
                else:
                    state = State.start
                    continue
            if state == State.command or state == State.command_all:
                cmd = self.ser.read(4).decode()
                cmd = c + cmd
                resp = state == State.command
                if cmd == "START":
                    print('remoterf start')
                    if not self._fight:
                        self.gameplay.enable()
                    self._fight = True
                    if resp:
                        self.ser.write(ack_packet)
                elif cmd == "STOP-":
                    print('remoterf stop')
                    if self._fight:
                        self.gameplay.disable()
                    self._fight = False
                    if resp:
                        self.ser.write(ack_packet)
                        print("ack", end=" ")
                elif cmd == "PING-":
                    print('remoterf ping')
                    self.ser.write(ack_packet)
                state = State.start
                self.ser.read(4) # read end of the packet
                continue

if __name__ == "__main__":
    rf = RemoteRF("/dev/ttyACM0", field_id='B', robot_id='A')
    rf.start()
    while True:
        #print(rf.fight)
        sleep(0.1)
    rf.stop()
