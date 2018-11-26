import json
import re
from time import time

import serial

import messenger
from config_manager import ConfigManager
from serial_wrapper import find_serial


class InjectorNode(messenger.Node):

    def __init__(self, mock=False, run=True) -> None:
        super().__init__('injector_node')

        self.strategy_listener = messenger.Listener(
            '/strategy', messenger.Messages.string)
        self.canbus_listener = messenger.Listener(
            '/canbus_message', messenger.Messages.string)
        self.settings_listener = messenger.Listener(
            '/settings_changed', messenger.Messages.string, callback=self.refresh_settings)
        self.recognition_listener = messenger.Listener(
            '/recognition', messenger.Messages.string)

        self.config = ConfigManager.get_value('game')
        self.mock = mock

        self.initial_devices = set(find_serial('CP2102').keys())
        self.injector = None

        if run:
            self.loop(3)

    def refresh_settings(self, settings_topic):
        self.config = ConfigManager.get_value('game')

    def find_serial(self):
        injector_serial = next(
            (device for device in find_serial('CP2102').keys() if device not in self.initial_devices),
            None,
        )
        if injector_serial:
            self.logger.info("Opening %s", injector_serial)

            self.injector = serial.Serial(
                port=injector_serial,
                baudrate=115200, xonxoff=True, timeout=0.01)

            self.sleep()
            self.read_config()

    def read_config(self):
        try:
            self.injector.write("config_package\n\r".encode("ascii"))
            self.sleep()
            config_string = self.injector.read(size=1024).decode('utf-8')
            self.loginfo("Config found: %s", str(config_string))
        except Exception as e:
            self.logerror("No config found, %s", e)
            return

        config = {}
        try:
            m = re.search('\!\@\#\$(.+?)\!\@\#\$', config_string)
            if m:
                config_string = m.group(1)
            config = json.loads(config_string)
        except Exception as e:
            self.logerror("Could not load settings package: %s", e)

        if config:
            for k, v in config.items():
                ConfigManager.set_value('game|global|%s' % k, v)

    def step(self):
        if not self.injector:
            self.find_serial()
        else:
            canbus_package = self.canbus_listener.package or {}
            strategy_package = self.strategy_listener.package or {}
            recognition_package = self.recognition_listener.package or {}

            package_A = (
                "{:<5}: {}".format('batt', canbus_package.get('voltage')),
                "{:<5}: {}".format('play', strategy_package.get('is_enabled')),
                "{:<5}: {}".format('goal', strategy_package.get('goal')),
                "{:<5}: {}".format('field', strategy_package.get('field')),
                "{:<5}: {}".format('robot', strategy_package.get('robot')),
                "{:<5}: {}".format('state', strategy_package.get('state')),
                "{:<5}: {}".format('dist', strategy_package.get('dist')),
                "{:<5}: {}".format('angle', strategy_package.get('angle')),
            )

            active_nodes = messenger.list()
            package_B = (
                "{:<8}: {}".format('recog_fps', f"{round(recognition_package.get('fps', 0))}fps"),
                "{:<8}: {}".format('recog_lat', f"{round(recognition_package.get('lat', 0) * 1000)}ms"),
                "{:<8}: {}".format('io_srv', '/io_server' in active_nodes),
                "{:<8}: {}".format('img_srv', '/image_server' in active_nodes),
                "{:<8}: {}".format('kicker', '/kicker_node' in active_nodes),
                "{:<8}: {}".format('motors', '/motion_node' in active_nodes),
                "{:<8}: {}".format('game', '/gameplay' in active_nodes),
                "{:<8}: {}".format('time', round(time()) % 1000),
            )

            try:
                command = "data=data if len(data)==2 else[[],[],]\n\r"
                self.injector.write(command.encode("ascii"))
                self.sleep()
                command = "data[0]=%s\n\r" % repr(package_A)
                self.injector.write(command.encode("ascii"))
                self.sleep()
                command = "data[1]=%s\n\r" % repr(package_B)
                self.injector.write(command.encode("ascii"))
                self.sleep()
            except Exception as e:
                self.logerror("Injector lost, %s", e)
                self.injector = None


if __name__ == '__main__':
    node = InjectorNode(mock=True, run=False)
    messenger.test()
    node.spin()
