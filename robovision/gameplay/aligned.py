
import logging
logger = logging.getLogger("gameplay")
from managed_threading import ManagedThread
from image_recognition import ImageRecognition
from controller import Controller
import math

class Gameplay(ManagedThread):
    """
    Gameplay that keeps robot aligned to towards the goal
    """
    def step(self, r, *args):
        """
        r.goal_yellow - Yellow goal relative to kicker, None if not detected
        r.goal_blue - Blue goal relative to kicker, None if not detected
        r.goal_blue.x
        r.goal_blue.y
        r.goal_blue.dist
        r.goal_blue.angle_deg

        r.robot - Robot's position on the field, None if not positioned
        r.orientation - Kicker's rotation along field grid, None if not positioned

        r.balls - List of recognized ball tuples, first element relative and second one absolute if robot is positioned
        """


        if not r.goal_blue:
            state = "No blue goal found, slowing down"
            self.rotation = self.rotation * 0.75
        else:
            state = "Found blue goal, keeping angle"
            self.rotation = -r.goal_blue.angle_deg / 180.0

        for relative, absolute, _, _, _ in r.balls:
            if abs(relative.angle_deg) > 80:
                state = "Going back"
                if relative.dist > 1.0:
                    if relative.angle_deg < 0:
                        self.x = 0.3
                    else:
                        self.x = -0.3
                else:
                    self.x = 0
                self.y = -0.5

            elif abs(relative.angle_deg) > 10:
                state = "Going to side"
                if relative.angle_deg > 0:
                    self.x = 0.3
                else:
                    self.x = -0.3
                self.y = 0
            elif r.ball_grabbed:
                self.arduino.kick()
                state = "Kicking ball"
                self.x = 0
                self.y = 0.8
            else:
                state = "Going foraward"
                self.x = 0
                self.y = 0.6
            break
        else:
            state = "No balls"
            self.y = 0.5
            self.x = 0.0

        if state != self.state and self.prev == state:
            self.arduino.set_xyw(self.x, self.y, self.rotation)
            logger.info(state)
            self.state = state
        self.prev = state

    def __init__(self, *args):
        ManagedThread.__init__(self, *args)
        self.rotation = 0.0
        self.arduino = Controller() # config read from ~/.robovision/pymata.conf
        self.rotation, self.x, self.y = 0, 0, 0.2
        self.state = None
        self.prev = None

    def start(self):
        self.arduino.start()
        ManagedThread.start(self)

