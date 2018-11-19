from typing import Callable

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32

from time import time, sleep


class Messages:
    motion = Twist
    string = String
    integer = Int32


class Reader:
    def __init__(self, topic: str, msg: Callable, callback: Callable = None) -> None:
        self.topic = topic
        self.msg = msg
        self.last_reading = None
        self.last_reading_time = 0
        self.callback = callback
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
        self.publisher = rospy.Publisher(topic, msg, queue_size=10)

    def publish(self, *args, **kwargs):
        self.publisher.publish(self.msg(*args, **kwargs))


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


if __name__ == '__main__':
    node = Node("messenger")
    reader = Reader("/messenger", Messages.motion)
    test()
