from time import time

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

    def reset(self):
        self.x = 0
        self.y = 0
        self.w = 0
        self.rpm = 0

    def apply(self):
        if self.x or self.y or self.w:
            self.movement_publisher.publish(x=self.x, y=self.y, az=self.w)
        if self.rpm:
            self.kicker_publisher.publish(int(self.rpm))


class GameplayNode(messenger.Node):

    def __init__(self, mock=False, run=True, **kwargs) -> None:
        super().__init__('gameplay', existing_loggers=['gameplay'], **kwargs)

        self.mock = mock
        # ConfigManager.set_value('game|global|gameplay status', 'disabled')
        self.config = ConfigManager.get_value('game')
        self.gameplay = Gameplay(self.config, Controller(), self.logger)

        self.strategy_publisher = messenger.Publisher('/strategy', messenger.Messages.string)

        self.settings_listener = messenger.Listener(
            '/settings_changed', messenger.Messages.string, callback=self.refresh_settings)
        self.recognition_listener = messenger.Listener(
            '/recognition', messenger.Messages.string, callback=self.callback)
        self.command_listener = messenger.Listener(
            '/command', messenger.Messages.string, callback=self.command_callback)
        self.kicker_listener = messenger.Listener(
            '/canbus_message', messenger.Messages.string, callback=self.kicker_callback)

        self.realsense_distance_listener = messenger.Listener(
            '/distance/realsense', messenger.Messages.float, callback=self.realsense_distance_callback)

        self.tfmini_distance_listener = messenger.Listener(
            '/distance/tfmini', messenger.Messages.float, callback=self.tfmini_distance_callback)

        self.realsense_active = time()

        self.logger.info("Start gameplay")
        if run:
            self.spin()

    def refresh_settings(self, settings_topic):
        self.loginfo("Settings refreshed")
        self.config = ConfigManager.get_value('game')
        self.gameplay.config = Settings(self.config)

    def get_recognition(self):
        package = self.recognition_listener.package
        if package:
            return RecognitionState.from_dict(package)

    def kicker_callback(self, *_):
        package = self.kicker_listener.package
        if package:
            self.gameplay.kicker_speed = package.get('rpm', 0)

    def realsense_distance_callback(self, *_):
        self.realsense_active = time()
        self.gameplay.target_goal_distance = self.realsense_distance_listener.last_reading.data

    def tfmini_distance_callback(self, *_):
        angle = self.gameplay.target_goal_angle
        if self.gameplay.alligned:
            timeout = time() - self.realsense_active > 1
            self.loginfo_throttle(2, f"TF-MINI is setting DISTANCE, RS:{not timeout} ANGLE:{angle}")
            if timeout:
                self.gameplay.target_goal_distance = self.tfmini_distance_listener.last_reading.data

    def command_callback(self, *_):
        package = self.command_listener.package
        if package:
            self.gameplay.motors.reset()

            r_state = self.get_recognition()
            self.gameplay.recognition = r_state
            self.gameplay.update_recent_closest_balls()
            self.gameplay.set_target_goal_distance()
            self.gameplay.set_target_goal_angle_adjust()

            for function_name, arguments in package.items():
                try:
                    func = getattr(self.gameplay, function_name)
                    func(**(arguments or {}))

                    self.logger.info_throttle(1, 'command success')
                except Exception as e:
                    self.logger.error('Gameplay command failed: %s %s\n %s', function_name, arguments, e)

            self.gameplay.motors.apply()

    def callback(self, *_):
        r_state = self.get_recognition()
        if r_state:
            self.gameplay.step(r_state)

            gp = self.gameplay
            self.strategy_publisher.command(
                is_enabled=gp.is_enabled,
                target_goal_angle=gp.target_goal_angle,
                goal=gp.config_goal,
                field=gp.field_id,
                robot=gp.robot_id,
                state=str(gp.state),
                dist=gp.target_goal_distance,
                angle=gp.target_goal_angle,
                average_closest_ball=gp.average_closest_ball and gp.average_closest_ball.serialize(),
                closest_ball=gp.closest_ball and gp.closest_ball.serialize(),
                id_balls=[bi.serialize() for bi in gp.sorted_id_balls],
                pwm=gp.get_desired_kicker_speed(),
            )

            # if self.gameplay.is_enabled:
            #     keys = tuple(package.keys())
            #     self.loginfo_throttle(2, f"PACK: {keys}")
            #


if __name__ == '__main__':
    node = GameplayNode(mock=True, run=False)
    messenger.test()
    node.spin()
