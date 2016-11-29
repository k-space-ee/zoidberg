
import logging
logger = logging.getLogger("gameplay")
from managed_threading import ManagedThread
from image_recognition import ImageRecognition
from arduino import Arduino
import math
from time import time

from collections import defaultdict


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

    @property
    def flank_is_alligned(self):
        # if self.balls and abs(self.balls[0][0].angle_deg) < 10:
        #     return True

        in_line = self.goal_to_ball_angle

        if in_line and abs(in_line) < 13:
            log_str = "B{:.1f} G{:.1f} | D{:.1f}".format(self.balls[0][0].angle_deg, self.target_goal.angle_deg, in_line)
            logger.info(log_str)

            return True

    @property
    def goal_to_ball_angle(self):
        import math

        if not self.target_goal or not self.balls:
            return

        ball = self.balls[0][0]
        goal = self.target_goal

        v1 = goal
        v2 = ball

        v1_theta = math.atan2(v1.y, v1.x)
        v2_theta = math.atan2(v2.y, v2.x)

        r = (v2_theta - v1_theta) * (180.0 / 3.141)

        if r > 180:
            r -= 360
        if r < -180:
            r += 360
        return r

    def flank_vector(self):
        r = self.goal_to_ball_angle
        if not r:
            return

        ball = self.balls[0][0]

        sign = [-1, 1][r>0]

        delta_deg = 45 * sign / ball.dist

        delta = math.radians(delta_deg)

        bx, by = ball.x, ball.y
        x = bx * math.cos(delta) - by * math.sin(delta)
        y = bx * math.sin(delta) + by * math.cos(delta)

        angle = math.degrees(math.atan2((y), (x)))
        # logger.info("delta{} angle{} ball{} -> {}".format(round(delta_deg),round(r),round(ball.angle_deg), round(angle)))

        return x,y

    def flank(self):
        flank = self.flank_vector()
        if not flank:
            return
        
        x, y = flank
        min_speed = 0.80
        max_val = max([abs(x), abs(y)])
        if max_val < min_speed and max_val:
            scaling = min_speed / max_val
            x *= scaling
            y *= scaling

        rotating_sign = [1,-1][self.target_goal_detla>0]
        angle = min(abs(self.target_goal_detla), 30)

        delta = rotating_sign * 0.8 * angle / 30

        self.arduino.set_xyw(y, x, delta)

    def kick(self):
        return self.arduino.kick()

    def stop_moving(self):
        self.arduino.set_abc(0,0,0)

    def drive_to_ball(self):

        if not self.balls:
            return 

        ball = self.balls[0][0]

        x, y = ball.x, ball.y
        min_speed = 0.5
        max_val = max([abs(x), abs(y)])
        if max_val < min_speed and max_val:
            scaling = min_speed / max_val
            x *= scaling
            y *= scaling

        w = - ball.angle_deg / 180.0

        self.arduino.set_xyw(y, x, w)

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
        self.timers = defaultdict(time)

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
            return Flank(self.actor)

    def VEC_HAS_BALL(self):
        if self.actor.has_ball:
            logger.info("Robot in %s VEC_HAS_BALL" % str(self) )
            return FindGoal(self.actor)

    def VEC_TOO_CLOSE(self):
        if self.actor.too_close:
            logger.info("Robot in %s VEC_TOO_CLOSE" % str(self))
            return Penalty(self.actor)


class Flank(StateNode):
    def animate(self):
        # if not self.actor.flank_is_alligned:
        self.actor.flank()
        # logger.info(self.actor.balls)

    def VEC_HAS_BALL(self):
        if self.actor.has_ball:
            logger.info("Robot in Flank VEC_HAS_BALL")
            return FindGoal(self.actor)

    def VEC_LOST_SIGHT(self):
        if not self.actor.balls:
            logger.info("Robot in Flank VEC_LOST_SIGHT")
            return Patrol(self.actor)

    def VEC_TOO_CLOSE(self):
        if self.actor.too_close:
            logger.info("Robot in %s VEC_TOO_CLOSE" % str(self))
            return Penalty(self.actor)

    def VEC_FLANK_DONE(self):
        self.timers["VEC_FLANK_DONE"] = self.timers["VEC_FLANK_DONE"]
        
        if self.actor.flank_is_alligned and time() + 0.3 > self.timers["VEC_FLANK_DONE"]:
            logger.info("Robot in %s VEC_FLANK_DONE" % str(self))
            return Drive(self.actor)

    #def VEC_LOST_GOAL_BUT_HAVE_BALL -> Drive?


class Drive(StateNode):
    def animate(self):
        self.actor.drive_to_ball()

    def VEC_HAS_BALL(self):
        if self.actor.has_ball:
            logger.info("Robot in drive VEC_HAS_BALL")
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
        logger.info("Blue {}; Yellow {}".format(self.actor.target_goal.dist,self.actor.own_goal.dist))

    def VEC_ENOUGH_FAR(self):
        if self.time + self.actor.penalty_time < time():
            logger.info("Robot in %s VEC_ENOUGH_FAR" % str(self))
            return Patrol(self.actor)
