import rospy
from geometry_msgs.msg import Twist

from time import time

from .controller import Controller

last_callback = time()


def callback(data):
    global last_callback
    start = time()

    linear = data.linear
    angular = data.angular

    x, y, z = linear.x, linear.y, linear.z
    ax, ay, az = angular.x, angular.y, angular.z

    controller.set_xyw(y, x, az)
    controller.apply()

    end = time()
    time_taken = end - start
    time_since_last = end - last_callback
    rospy.loginfo(
        ["linear", [x, y, z], "angular", [ax, ay, az], "time taken", time_taken, "time since", time_since_last])
    last_callback = time()

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/movement", Twist, callback)
    rospy.spin()


if __name__ == '__main__':
    controller = Controller()
    listener()
