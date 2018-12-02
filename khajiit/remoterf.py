import logging
import serial

import messenger
from config_manager import ConfigManager
from serial_wrapper import find_serial

logger = logging.getLogger('remoterf')


class RemoteRF(messenger.Node):
    def __init__(self, mock=False, run=True, silent=True) -> None:
        super().__init__('remoterf', existing_loggers=['remoterf', 'config_manager'])

        self.silent = silent
        self.mock = mock

        path = next(iter(find_serial('FT232R').keys()), None)

        self.ser = serial.Serial(path, timeout=1)
        if run:
            self.run()

    def run(self):
        logger.info("REMOTERF THREAD STARTED")
        string = ""
        while True:
            field = ConfigManager.get_value("game|global|field_id")
            ack_packet = "a{}{}ACK------".format(
                ConfigManager.get_value("game|global|field_id"),
                ConfigManager.get_value("game|global|robot_id"),
            )
            ack_packet = ack_packet.encode()
            c = self.ser.read(11)

            # ---aAXSTART -
            self.ser.write(ack_packet)

            try:
                c = c.decode()
            except:
                continue
            if not c:
                continue
            string += c

            if f"{field}XSTART" in string:
                logger.info("GOT START")
                string = ""
                ConfigManager.set_value("game|global|gameplay status", 'enabled')

            if f"{field}XSTOP" in string:
                logger.info("GOT STOP")
                string = ""
                ConfigManager.set_value("game|global|gameplay status", 'disabled')

            if "PING" in string:
                logger.info("got ping")
                string = ""
                self.ser.write(ack_packet)

            if len(string) > 1000:
                string = ""


if __name__ == "__main__":
    rf = RemoteRF()
