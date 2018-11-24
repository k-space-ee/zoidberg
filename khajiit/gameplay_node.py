import messenger
from config_manager import ConfigManager, Settings
from gameplay import Gameplay, RecognitionState


class Controller:

    def __init__(self) -> None:
        self.movement_publisher = messenger.Publisher('/movement', messenger.Messages.motion)
        self.kicker_publisher = messenger.Publisher('/kicker_speed', messenger.Messages.integer)
        self.x = 0
        self.y = 0
        self.w = 0
        self.rpm = 0

    def start(self) -> None:
        self.set_xyw(0, 0, 0)
        self.set_grabber()
        self.apply()

    def set_xyw(self, x: float, y: float, w: float) -> None:
        self.x = x
        self.y = y
        self.w = w

    def set_thrower(self, rpm: int):
        self.rpm = rpm

    def set_grabber(self):
        self.rpm = 550

    def apply(self):
        self.movement_publisher.publish(x=self.x, y=self.y, az=self.w)
        self.kicker_publisher.publish(int(self.rpm))


class GameplayNode(messenger.Node):

    def __init__(self, mock=False, run=True, **kwargs) -> None:
        super().__init__('gameplay', existing_loggers=['gameplay'], **kwargs)
        self.listener = messenger.Listener('/recognition', messenger.Messages.string, callback=self.callback)

        self.settings_listener = messenger.Listener(
            '/settings_changed', messenger.Messages.string, callback=self.refresh_settings)

        self.recognition_listener = messenger.Listener('/recognition', messenger.Messages.string,
                                                       callback=self.callback)
        self.mock = mock
        self.config = ConfigManager.get_value('game')
        self.gameplay = Gameplay(self.config, Controller())

        self.logger.info("Start gameplay")
        if run:
            self.spin()

    def refresh_settings(self, settings_topic):
        self.loginfo("Settings refreshed")
        self.config = ConfigManager.get_value('game')
        self.gameplay.config = Settings(self.config)

    def callback(self, *_):
        package = self.listener.package
        if package:
            keys = tuple(package.keys())
            self.loginfo_throttle(0.5, f"PACK: {keys}")

            r_state = RecognitionState.from_dict(package)
            self.gameplay.step(r_state)


if __name__ == '__main__':
    node = GameplayNode(mock=True, run=False)
    messenger.test()
    node.spin()
