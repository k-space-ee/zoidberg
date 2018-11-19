from typing import Callable

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32

from time import time, sleep


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


class Reader:
    def __init__(self, topic: str, msg: Callable, callback: Callable = None) -> None:
        self.topic = topic
        self.msg = msg
        self.last_reading = None
        self.last_reading_time = 0
        self.callback = callback

        msg = getattr(msg, 'message', msg)
        self.subscriber = rospy.Subscriber(topic, msg, self.recieve, queue_size=10)

    def recieve(self, data):
        self.last_reading = data
        self.last_reading_time = time()
        if self.callback:
            self.callback()


class Publisher:
    def __init__(self, topic: str, msg: Callable) -> None:
        self.topic = topic
        self.msg = msg
        self.last_reading = None
        self.last_reading_time = 0
        msg = getattr(msg, 'message', msg)
        self.publisher = rospy.Publisher(topic, msg, queue_size=10)

    def publish(self, *args, **kwargs):
        message = self.msg(*args, **kwargs)
        msg = getattr(message, 'msg', message)
        self.publisher.publish(msg)


class Node:
    def __init__(self, name: str) -> None:
        self.node = rospy.init_node(name, anonymous=True)
        self.logger = rospy.loginfo

    @staticmethod
    def spin():
        rospy.spin()


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


def core():
    import os
    os.system('roscore')  # lives and dies with this process


if __name__ == '__main__':
    node = Node("messenger")
    reader = Reader("/messenger", Messages.motion)
    test()
