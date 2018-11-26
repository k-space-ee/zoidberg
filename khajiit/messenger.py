import json
import logging
from typing import Callable, Dict, Optional, List

import rospy
import rosnode
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32
from rosgraph_msgs.msg import Log

from time import time, sleep


# ROS: the R stands **tarded
# https://github.com/ros/ros_comm/issues/1384
# https://gist.github.com/nzjrs/8712011
class ConnectPythonLoggingToROS(logging.Handler):
    MAP = {
        logging.DEBUG: rospy.logdebug,
        logging.INFO: rospy.loginfo,
        logging.WARNING: rospy.logwarn,
        logging.ERROR: rospy.logerr,
        logging.CRITICAL: rospy.logfatal
    }

    def emit(self, record):
        try:
            self.MAP[record.levelno]("%s: %s" % (record.name, record.msg), *record.args)
        except KeyError:
            rospy.logerr("unknown log level %s LOG: %s: %s" % (record.levelno, record.name, record.msg))

    @classmethod
    def reconnect(cls, *loggers):
        key: str
        logger: logging.Logger
        for key, logger in logging.Logger.manager.loggerDict.items():
            if key not in loggers:
                continue

            has_handlers = getattr(logger, 'handlers', None)
            if has_handlers is not None and not key.startswith('ros') and logger.__module__ == 'rosgraph.roslogging':

                # reconnect logging calls which are children of this to the ros log system
                if not any(isinstance(handler, cls) for handler in logger.handlers):
                    logger.addHandler(cls())
                    logger.info("Logger directed to ROS")


class CallbackListenerWrapper:

    def __init__(self) -> None:
        self.listeners = []

    def update(self, *args):
        for func in self.listeners:
            func(*args)


class TwistWrapper:
    message = Twist

    def __init__(self, x=0, y=0, z=0, ax=0, ay=0, az=0) -> None:
        self.msg = self.message()
        self.msg.linear.x = x
        self.msg.linear.y = y
        self.msg.linear.z = z
        self.msg.angular.x = ax
        self.msg.angular.y = ay
        self.msg.angular.z = az


class Messages:
    motion = TwistWrapper
    string = String
    integer = Int32
    logging = Log


class LoggerWrapper:
    debug = rospy.logdebug
    debug_throttle = rospy.logdebug_throttle
    info = rospy.loginfo
    info_throttle = rospy.loginfo_throttle
    warning = rospy.logwarn
    warning_throttle = rospy.logwarn_throttle
    error = rospy.logerr
    error_throttle = rospy.logerr_throttle
    critical = rospy.logfatal
    critical_throttle = rospy.logfatal_throttle


class Listener:
    def __init__(self, topic: str, msg: Callable, callback: Callable = None) -> None:
        self.topic = topic
        self.msg = msg
        self.last_reading = None
        self.last_reading_time = 0
        self.callback = callback

        msg = getattr(msg, 'message', msg)
        self.subscriber = rospy.Subscriber(topic, msg, self.receive, queue_size=10)
        self.logger = LoggerWrapper

    def receive(self, data):
        self.last_reading = data
        self.last_reading_time = time()
        if self.callback:
            self.callback(data)

    @property
    def package(self) -> Optional[Dict]:
        if not self.last_reading or not issubclass(self.msg, Messages.string):
            return None
        try:
            return json.loads(self.last_reading.data)
        except Exception as e:
            rospy.logerr_throttle(0.5, f'Parse package failed:\n{e}\n{self.last_reading}')
            return None


class Publisher:
    def __init__(self, topic: str, msg: Callable) -> None:
        self.topic = topic
        self.msg = msg
        self.last_reading = None
        self.last_reading_time = 0
        msg = getattr(msg, 'message', msg)
        self.publisher = rospy.Publisher(topic, msg, queue_size=10)
        self.logger = LoggerWrapper

    def publish(self, *args, **kwargs):
        message = self.msg(*args, **kwargs)
        msg = getattr(message, 'msg', message)
        self.publisher.publish(msg)

    def command(self, **commands):
        assert self.msg == Messages.string, 'Commands available only on Messages.string mode'
        command = json.dumps(commands, indent=1)
        self.publish(command)


class Node:
    def __init__(self, name: str, disable_signals=True, existing_loggers=None) -> None:
        # TODO: disabled signals so that the damn rosnodes would die peacefully
        self.node = rospy.init_node(name, anonymous=False, disable_signals=disable_signals)

        self.logdebug = LoggerWrapper.debug
        self.logdebug_throttle = LoggerWrapper.debug_throttle
        self.loginfo = LoggerWrapper.info
        self.loginfo_throttle = LoggerWrapper.info_throttle
        self.logwarning = LoggerWrapper.warning
        self.logwarning_throttle = LoggerWrapper.warning_throttle
        self.logerror = LoggerWrapper.error
        self.logerror_throttle = LoggerWrapper.error_throttle
        self.logcritical = LoggerWrapper.critical
        self.logcritical_throttle = LoggerWrapper.critical_throttle
        self.logger = LoggerWrapper

        self.register_existing_loggers(*(existing_loggers or []))
        self._rate = None

    @staticmethod
    def register_existing_loggers(*loggers):
        ConnectPythonLoggingToROS.reconnect(*loggers)

    def spin(self):
        rospy.spin()

    def rate(self, hz):
        self._rate = rospy.Rate(hz)

    def sleep(self):
        self._rate and self._rate.sleep()

    def loop(self, hz=10):
        self.rate(hz)
        while not rospy.is_shutdown():
            self.step()
            self._rate.sleep()

    def step(self):
        raise NotImplemented("Can't run, please implement Node.step")


class Timer:

    def __init__(self, count: int, callback: Callable = None) -> None:
        self.measurements = []
        self.count = count
        self.avg = -1
        self.callback = callback

    def __call__(self, function: Callable, *args, **kwargs):
        start = time()
        function(*args, **kwargs)
        end = time() - start
        self.measurements.append(end)
        self.measurements = self.measurements[-self.count:]
        self.avg = sum(self.measurements) / len(self.measurements)

        if self.callback:
            self.callback()

        print(self)

    def __str__(self):
        return str(self.avg)


def test():
    import os
    sleep(1)
    print("ROSNODE")
    print(os.popen("rosnode list").read())
    print("ROSTOPIC")
    print(os.popen("rostopic list").read())
    print("END")


def list() -> List[str]:
    # no freaking api for getting alive nodes, greato!
    active_nodes = rosnode.get_node_names()
    active_nodes = [node for node in active_nodes if rosnode.rosnode_ping(node, max_count=1)]
    return active_nodes


def core():
    import os
    os.system('roscore')  # lives and dies with this process


if __name__ == '__main__':
    node = Node("messenger")
    reader = Listener("/messenger", Messages.motion)
    test()
