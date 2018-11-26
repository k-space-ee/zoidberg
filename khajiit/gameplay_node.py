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

        self.strategy_publisher = messenger.Publisher('/strategy', messenger.Messages.string)

        self.settings_listener = messenger.Listener(
            '/settings_changed', messenger.Messages.string, callback=self.refresh_settings)
        self.recognition_listener = messenger.Listener(
            '/recognition', messenger.Messages.string, callback=self.callback)
        self.command_listener = messenger.Listener(
            '/command', messenger.Messages.string, callback=self.command_callback)

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

    def command_callback(self, *_):
        package = self.command_listener.package
        if package:
            for function_name, arguments in package.items():
                try:
                    func = getattr(self.gameplay, function_name)
                    func(**arguments)
                    self.logger.info_throttle(1, 'command success')
                except Exception as e:
                    self.logger.error('Gameplay command failed: %s %s', function_name, arguments)

    def callback(self, *_):
        package = self.recognition_listener.package
        if package:
            r_state = RecognitionState.from_dict(package)
            self.gameplay.step(r_state)

            self.strategy_publisher.command(
                is_enabled=self.gameplay.is_enabled,
                target_goal_angle=self.gameplay.target_goal_angle,
                goal=self.gameplay.config_goal,
                field=self.gameplay.field_id,
                robot=self.gameplay.robot_id,
                state=str(self.gameplay.state),
                dist=self.gameplay.get_target_goal_distance(),
                angle=self.gameplay.target_goal_angle,
            )

            if self.gameplay.is_enabled:
                keys = tuple(package.keys())
                self.loginfo_throttle(2, f"PACK: {keys}")


if __name__ == '__main__':
    node = GameplayNode(mock=True, run=False)
    messenger.test()
    node.spin()
