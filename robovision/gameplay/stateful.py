
import logging
logger = logging.getLogger("gameplay")
from managed_threading import ManagedThread
from image_recognition import ImageRecognition
from arduino import Arduino
import math
from time import time


class Gameplay(ManagedThread): 
    def __init__(self, *args):        
        ManagedThread.__init__(self, *args)
        self.arduino = Arduino() # config read from ~/.robovision/pymata.conf
        self.state = Patrol(self)
        self.recognition = None

    @property
    def balls(self):
        return self.recognition.balls

    @property
    def own_goal(self):
        return self.recognition.goal_yellow # TODO: imagerecogn.own_goal

    @property
    def target_goal(self):
        return self.recognition.goal_blue # TODO: imagerecogn.target_goal

    @property
    def has_ball(self):
        return self.arduino.board and not self.arduino.board.digital_read(13)

    @property
    def target_goal_detla(self):
        if self.target_goal:
            return self.target_goal.angle_deg
    
    @property
    def target_goal_dist(self):
        if self.target_goal:
            return self.target_goal.dist

    @property
    def too_close(self):
        min_dist = 0.30
        if self.target_goal and self.target_goal.dist < min_dist:
            return True
        if self.own_goal and self.own_goal.dist < min_dist:
            return True

    @property
    def alligned(self):
        # TODO: inverse of this
        if self.target_goal:
            min_angle = 2 if not self.target_goal_dist or self.target_goal_dist>3 else 4
            return abs(self.target_goal_detla) <= min_angle 

    @property
    def focus_time(self):
        return 0.1

    @property
    def rest_time(self):
        return 0.1

    @property
    def penalty_time(self):
        return 2

    def align_to_target_goal(self):
        if not self.target_goal:
            return

        if self.target_goal.dist > 0.5:
            for relative, _, _, _, _ in self.balls[1:]:
                if abs(relative.dist*math.sin(relative.angle))<0.08 and abs(relative.angle_deg) < 60 and abs(relative.dist - self.target_goal.dist):
                    self.arduino.set_xyw([1,-1][relative.angle>0]*0.60,0.0,0.0)
                    return

        rotating_sign = [1,-1][self.target_goal_detla>0]
        delta = rotating_sign * (0.8 if abs(self.target_goal_detla) > 20 else 0.5)
        if delta < 0:
            self.arduino.set_abc(delta * 0.05, delta * 0.1, delta*0.8)
        elif delta > 0:
            self.arduino.set_abc(delta * 0.1, delta * 0.05, delta*0.8)

    def kick(self):
        return self.arduino.kick()

    def stop_moving(self):
        self.arduino.set_abc(0,0,0)

    def drive_to_ball(self):

        if not self.balls:
            return 

        ball = self.balls[0][0]
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

        min_speed = 0.40
        max_val = max([abs(x), abs(y)])
        if max_val < min_speed and max_val:
            scaling = min_speed / max_val
            x *= scaling
            y *= scaling

        self.arduino.set_xyw(x, y, w)

    def drive_to_own_goal(self):

        # logger.info(str([self.own_goal]))
        if not self.own_goal:
            return 

        x, y = self.own_goal.x, self.own_goal.y

        if self.own_goal.dist < 1:
            x *= -1
            y *= -1
        
        self.arduino.set_xyw(y, x, 0)

    def step(self, recognition, *args):
        if not recognition:
            return

        self.recognition = recognition
        self.state = self.state.tick()

    def on_enabled(self, *args):
        self.arduino.board.digital_write(12, 1)
        pass

    def on_disabled(self, *args):
        self.arduino.board.digital_write(12, 0)
        self.arduino.set_xyw(0, 0, 0)

    def start(self):
        self.arduino.start()
        ManagedThread.start(self)


class StateNode:
    def __init__(self,actor):
        self.transitions = dict(self.exctract_transistions())
        self.actor = actor
        self.time = time()

    def exctract_transistions(self):
        return [i for i in self.__class__.__dict__.items() if 'VEC' in i[0] ]

    def transition(self):        
        for name, vector in self.transitions.items():
            result = vector(self)
            if result:
                print('\n',name,'->',result.__class__.__name__)
                return result
        return self

    def animate(self):
        print("I exist")

    def tick(self):        
        next_state = self.transition() or self
        if next_state != self:
            self.actor.stop_moving() # PRE EMPTIVE, should not cause any issues TM
            logger.info(str(next_state))
        else:  #  TODO: THis causes latency, maybe ?
            self.animate() 
        return next_state

    def __str__(self):
        return str(self.__class__.__name__)


class Patrol(StateNode):
    def animate(self):
        pass

    def VEC_SEE_BALLS(self):
        if self.actor.balls:
            logger.info("Robot in %s VEC_SEE_BALLS" % str(self))
            return Drive(self.actor)

    def VEC_HAS_BALL(self):
        if self.actor.has_ball:
            logger.info("Robot in %s VEC_HAS_BALL" % str(self) )
            return FindGoal(self.actor)

    def VEC_TOO_CLOSE(self):
        if self.actor.too_close:
            logger.info("Robot in %s VEC_TOO_CLOSE" % str(self))
            return Penalty(self.actor)


class Drive(StateNode):
    def animate(self):
        self.actor.drive_to_ball()

    def VEC_HAS_BALLS(self):
        if self.actor.has_ball:
            logger.info("Robot in drive VEC_HAS_BALLS")
            return FindGoal(self.actor)

    def VEC_LOST_SIGHT(self):
        if not self.actor.balls:
            logger.info("Robot in drive VEC_LOST_SIGHT")
            return Patrol(self.actor)

    def VEC_TOO_CLOSE(self):
        if self.actor.too_close:
            logger.info("Robot in %s VEC_TOO_CLOSE" % str(self))
            return Penalty(self.actor)


class FindGoal(StateNode):
    def animate(self):
        self.actor.drive_to_own_goal()

    def VEC_HAS_GOAL(self):
        if self.actor.target_goal:
            logger.info("Robot in FindGoal VEC_HAS_GOAL")
            return TargetGoal(self.actor)

    def VEC_LOST_BALL(self):
        # logger.info("Robot in FindGoal BALL STATE {}".format(self.actor.has_ball))
        if not self.actor.has_ball:
            logger.info("Robot in FindGoal VEC_LOST_BALL")
            return Patrol(self.actor)


class TargetGoal(StateNode):
    def animate(self):
        self.actor.align_to_target_goal()

    def VEC_POINTED_AT_GOAL(self):
        # return
        if self.actor.alligned:
            logger.info("Robot in %s VEC_POINTED_AT_GOAL" % str(self))
            return Focus(self.actor)

    def VEC_LOST_BALL(self):
        if not self.actor.has_ball:
            logger.info("Robot in %s VEC_LOST_BALL" % str(self))
            return Patrol(self.actor)

    def VEC_LOST_GOAL(self):
        # logger.info("Robot in FindGoal BALL STATE {}".format(self.actor.has_ball))
        if not self.actor.target_goal:
            logger.info("Robot in %s VEC_LOST_GOAL" % str(self))
            return FindGoal(self.actor)

    def VEC_TOO_CLOSE(self):
        if self.actor.too_close:
            logger.info("Robot in %s VEC_TOO_CLOSE" % str(self))
            return Patrol(self.actor)


class Focus(StateNode):

    def animate(self):
        self.actor.stop_moving()
        pass

    def VEC_NOT_ALLIGNED(self):
        if not self.actor.alligned:
            logger.info("Robot in %s VEC_NOT_ALLIGNED" % str(self))
            return TargetGoal(self.actor)

    def VEC_LOST_BALL(self):
        if not self.actor.has_ball:
            logger.info("Robot in %s VEC_LOST_BALL" % str(self))
            return Patrol(self.actor)

    def VEC_READY_TO_SHOOT(self):
        if self.actor.alligned and self.actor.has_ball and self.time + self.actor.focus_time < time():
            logger.info("Robot in %s VEC_READY_TO_SHOOT" % str(self))
            return Shoot(self.actor)


class Shoot(StateNode):

    def animate(self):
        self.actor.kick()

    def VEC_DONE_SHOOTING(self):
        if not self.actor.has_ball:
            logger.info("Robot in %s VEC_DONE_SHOOTING" % str(self))
            return PostShoot(self.actor)

class PostShoot(StateNode):

    def animate(self):
        pass

    def VEC_DONE_LOLLYGAGGING(self):
        if self.time + self.actor.focus_time < time():
            logger.info("Robot in %s VEC_DONE_LOLLYGAGGING" % str(self))
            return Patrol(self.actor)


class Penalty(StateNode):

    def animate(self):
        self.actor.drive_to_own_goal()

    def VEC_ENOUGH_FAR(self):
        if self.time + self.actor.penalty_time < time():
            logger.info("Robot in %s VEC_ENOUGH_FAR" % str(self))
            return Patrol(self.actor)

# class Gameplay(ManagedThread):
#     """
#     Naive rotating gameplay
#     """
#     def drive_to_ball(self, ball):
#         '''
#         ball - position of the ball in relation to the robot
#         ball.angle_deg
#         ball.angle
#         ball.dist
#         ball.x
#         ball.y
#         '''
#         if abs(ball.angle_deg) < 30:
#             j = -ball.angle_deg/180
#         elif -ball.angle_deg < 0:
#             j = -0.3
#         elif -ball.angle_deg > 0:
#             j = 0.3
#         else:
#             j = 0
#         if j > 0 and j < 0.2:
#             j = 0.1
#         if j < 0 and j > -0.2:
#             j = -0.1

#         x, y, w = (
#             ball.y/ball.dist*max(min(ball.dist, 0.99), 0.2),
#             ball.x/ball.dist*max(min(ball.dist, 0.99), 0.2),
#             j
#         )
#         old = list([x,y,w])

#         min_speed = 0.65
#         max_val = max([abs(x), abs(y)])
#         if max_val < min_speed and max_val:
#             scaling = min_speed / max_val
#             x *= scaling
#             y *= scaling

#         logger.info(str(("SPEEDS", x,y,w, old)))
#         self.arduino.set_xyw(x, y, w)

#     def drive_to_next_ball(self, balls):
#         if len(balls)>1:
#             self.keep_state_counter=5
#             self.drive_to_ball(balls[1][0])
#             new_state = "repeat_last_move"
#         else:
#             new_state = "cant_find_any_balls"
#             #TODO drive somewhere to find ball
#         return new_state

#     def try_to_kick(self, r):
#         print('Inside try to kick')
#         for relative, absolute, _, _, _ in r.balls[1:]:
#             if abs(relative.dist*math.sin(relative.angle))<0.08:
#                 self.arduino.set_xyw([1,-1][relative.angle>0]*0.99,0.0,0.0)
#                 self.keep_state_counter=10
#                 new_state = "repeat_last_move"
#                 break
#         else:
#             print('kicking')
#             self.arduino.kick()
#             new_state = self.drive_to_next_ball(r.balls)
#         return new_state

#     def step(self, r, *args):
#         """
#         r.goal_yellow - Yellow goal relative to kicker, None if not detected
#         r.goal_blue - Blue goal relative to kicker, None if not detected
#         r.goal_blue.x
#         r.goal_blue.y
#         r.goal_blue.dist
#         r.goal_blue.angle_deg
#         r.robot - Robot's position on the field, None if not positioned
#         r.orientation - Kicker's rotation along field grid, None if not positioned
#         r.balls - List of recognized ball tuples, first element relative and second one absolute if robot is positioned
#         """
#         state = ''
#         if self.state ==  'repeat_last_move' and self.keep_state_counter > -1:
#             print('repeat last move'+str(self.keep_state_counter))
#             self.keep_state_counter -= 1
#             state = self.state
#         elif self.arduino.board and not self.arduino.board.digital_read(13):
#             if r.goal_blue:
#                 min_angle = 2 if not r.goal_blue.dist or r.goal_blue.dist>3 else 4
#                 if abs(r.goal_blue.angle_deg) <= min_angle:
#                     if  self.state == "waiting_for_camera_delay" :
#                         state = self.state
#                         self.keep_state_counter -= 1
#                         print('waiting_for_camera_delay: ' + str(self.keep_state_counter))
#                         if self.keep_state_counter <= 1:
#                             state = self.try_to_kick(r)
#                     else:
#                         self.arduino.set_abc(0,0,0)
#                         state = "waiting_for_camera_delay"
#                         self.keep_state_counter=4
#                 else:
#                     rotating_sign = [1,-1][r.goal_blue.angle_deg>0]
#                     delta = rotating_sign * (0.8 if abs(r.goal_blue.angle_deg) > 20 else 0.15)
#                     if delta < 0:
#                         self.arduino.set_abc(delta * 0.05, delta * 0.1, delta*0.8)
#                     elif delta > 0:
#                         self.arduino.set_abc(delta * 0.1, delta * 0.05, delta*0.8)
#                     state = "Got the ball, found goal, turning towards goal. Delta: "+ str(delta)
#             else:
#                 #TODO
#                 state = "Got the ball, looking for goal"
#         else:
#             for relative, absolute, _, _, _ in r.balls:
#                 self.drive_to_ball(relative)
#                 state = "Turning and going towards the ball. Ball count: "+str(len(r.balls))
#                 break
#             else:
#                 if r.robot:
#                     state = "Driving towards the center of the field"
#                     if r.orientation > 0 and r.orientation < math.pi:
#                         self.arduino.set_xyw((r.robot.x-2.3)*0.5, r.robot.y*0.5, 0.25)
#                     else:
#                         self.arduino.set_xyw((r.robot.x-2.3)*0.5, r.robot.y*0.5, -0.25)
#                 else:
#                     state = "Could not find any balls and I don't know where I am"
#                     self.arduino.set_xyw(0,0,0)
#         if state != self.state:
#             logger.info(state)
#             self.state = state

#     def on_enabled(self, *args):
#         self.arduino.board.digital_write(12, 1)


#     def on_disabled(self, *args):
#         self.arduino.board.digital_write(12, 0)
#         self.arduino.set_xyw(0, 0, 0)


#     def __init__(self, *args):
#         self.keep_state_counter = 0
#         ManagedThread.__init__(self, *args)
#         self.arduino = Arduino() # config read from ~/.robovision/pymata.conf
#         self.state = None

#     def start(self):
#         self.arduino.start()
#         ManagedThread.start(self)


