
import logging
logger = logging.getLogger("gameplay")
from managed_threading import ManagedThread
from image_recognition import ImageRecognition
from arduino import Arduino
import math
import time

class Gameplay(ManagedThread):
    """
    Naive rotating gameplay
    """
    def drive_to_ball(self, ball):
        '''
        ball - position of the ball in relation to the robot
        ball.angle_deg
        ball.angle
        ball.dist
        ball.x
        ball.y
        '''
        if abs(ball.angle_deg) < 30:
            j = -ball.angle_deg/180
        elif -ball.angle_deg < 0:
            j = -0.3
        elif -ball.angle_deg > 0:
            j = 0.3
        else:
            j = 0
        if j > 0 and j < 0.2:
            j = 0.1
        if j < 0 and j > -0.2:
            j = -0.1

        x, y, w = (
            ball.y/ball.dist*max(min(ball.dist, 0.99), 0.2),
            ball.x/ball.dist*max(min(ball.dist, 0.99), 0.2),
            j
        )
        old = list([x,y,w])

        min_speed = 0.65
        max_val = max([abs(x), abs(y)])
        if max_val < min_speed and max_val:
            scaling = min_speed / max_val
            x *= scaling
            y *= scaling

        logger.info(str(("SPEEDS", x,y,w, old)))
        self.arduino.set_xyw(x, y, w)

    def drive_to_next_ball(self, balls):
        if len(balls)>1:
            self.keep_state_counter=5
            self.drive_to_ball(balls[1][0])
            new_state = "repeat_last_move"
        else:
            new_state = "cant_find_any_balls"
            #TODO drive somewhere to find ball
        return new_state

    def try_to_kick(self, r):
        print('Inside try to kick')
        for relative, absolute, _, _, _ in r.balls[1:]:
            if abs(relative.dist*math.sin(relative.angle_rad))<0.08:
                self.arduino.set_xyw([1,-1][relative.angle_rad>0]*0.99,0.0,0.0)
                self.keep_state_counter=10
                new_state = "repeat_last_move"
                break
        else:
            print('kicking')
            self.arduino.kick()
            new_state = self.drive_to_next_ball(r.balls)
        return new_state

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
        #return
        state = ''
        if self.state ==  'repeat_last_move' and self.keep_state_counter > -1:
            print('repeat last move'+str(self.keep_state_counter))
            self.keep_state_counter -= 1
            state = self.state
        elif self.arduino.board and not self.arduino.board.digital_read(13):
            if r.goal_blue:
                min_angle=2 if not r.goal_blue.dist or r.goal_blue.dist>3 else 4
                if abs(r.goal_blue.angle_deg) <= min_angle:
                    if  self.state == "waiting_for_camera_delay" :
                        state = self.state
                        self.keep_state_counter -= 1
                        print('waiting_for_camera_delay: ' + str(self.keep_state_counter))
                        if self.keep_state_counter <= 1:
                            state = self.try_to_kick(r)
                    else:
                        self.arduino.set_abc(0,0,0)
                        state = "waiting_for_camera_delay"
                        self.keep_state_counter=4
                else:
                    rotating_sign = [1,-1][r.goal_blue.angle_deg>0]
                    delta = rotating_sign * (0.8 if abs(r.goal_blue.angle_deg) > 20 else 0.5)
                    if delta < 0:
                        self.arduino.set_abc(delta * 0.05, delta * 0.1, delta*0.8)
                    elif delta > 0:
                        self.arduino.set_abc(delta * 0.1, delta * 0.05, delta*0.8)
                    state = "Got the ball, found goal, turning towards goal. Delta: "+ str(delta)
            else:
                #TODO
                state = "Got the ball, looking for goal"
        else:
            for relative, absolute, _, _, _ in r.balls:
                self.drive_to_ball(relative)
                state = "Turning and going towards the ball. Ball count: "+str(len(r.balls))
                break
            else:
                if r.robot:
                    state = "Driving towards the center of the field"
                    if r.orientation > 0 and r.orientation < math.pi:
                        self.arduino.set_xyw((r.robot.x-2.3)*0.5, r.robot.y*0.5, 0.25)
                    else:
                        self.arduino.set_xyw((r.robot.x-2.3)*0.5, r.robot.y*0.5, -0.25)
                else:
                    state = "Could not find any balls and I don't know where I am"
                    self.arduino.set_xyw(0,0,0)
        if state != self.state:
            logger.info(state)
            self.state = state

    def on_enabled(self, *args):
        self.arduino.board.digital_write(12, 1)


    def on_disabled(self, *args):
        self.arduino.board.digital_write(12, 0)
        self.arduino.set_xyw(0, 0, 0)


    def __init__(self, *args):
        self.keep_state_counter = 0
        ManagedThread.__init__(self, *args)
        self.arduino = Arduino() # config read from ~/.robovision/pymata.conf
        self.state = None

    def start(self):
        self.arduino.start()
        ManagedThread.start(self)

