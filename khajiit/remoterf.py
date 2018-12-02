import logging
from threading import Thread
import serial

import messenger
from config_manager import ConfigManager
from serial_wrapper import find_serial

logger = logging.getLogger('remoterf')


class RemoteRF(Thread):
    def __init__(self, dev, gameplay=None):
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
        logger.info("REMOTERF THREAD STARTED")
        string = ""

        while not self._stop:
            logger.info("step")
            ack_packet = "a{}{}ACK------".format(
                ConfigManager.get_value("game|global|field_id"),
                ConfigManager.get_value("game|global|robot_id"),
            )
            ack_packet = ack_packet.encode()
            c = self.ser.read(3)

            self.ser.write(ack_packet)

            try:
                c = c.decode()
            except:
                continue
            if not c:
                continue
            string += c
            logger.info(string)

            if "START" in string:
                logger.info("GOT START")
                string = ""
                ConfigManager.set_value("game|global|gameplay status", 'enabled'),

            if "STOP" in string:
                logger.info("GOT STOP")
                string = ""
                ConfigManager.set_value("game|global|gameplay status", 'disabled'),

            if "PING" in string:
                logger.info("got ping")
                string = ""
                self.ser.write(ack_packet)

            if len(string) > 1000:
                string = ""


if __name__ == "__main__":
    node = messenger.Node('rf_test', disable_signals=True, existing_loggers=['remoterf'])
    path = next(iter(find_serial('FT232R').keys()), None)
    rf = RemoteRF(path)
    rf.start()

    rf.join()
    rf.stop()
