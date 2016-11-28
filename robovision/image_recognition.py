
import cv2
import numpy as np
import math
import time
import logging
from managed_threading import ManagedThread
logger = logging.getLogger("image_recognition")

class Point(object):
    """
    Cartesian coordinate (meters)
    """
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.angle = self.angle_rad = math.atan2(x, y)
        self.angle_deg = math.degrees(self.angle_rad)
        self.dist = (x**2 + y**2)**0.5

    def __iter__(self):
        yield self.x
        yield self.y

    def __add__(self, other_point):
        return self.translate(other_point)

    def __sub__(self, other_point):
        return Point(self.x - other_point.x, self.y - other_point.y)

    def __lshift__(self, angle_rad):
        return self.rotate(-angle_rad)

    def __rshift__(self, angle_rad):
        return self.rotate(angle_rad)

    def rotate(self, angle_rad):
        return Point(
            self.x * math.cos(angle_rad) - self.y * math.sin(angle_rad),
            self.x * math.sin(angle_rad) + self.y * math.cos(angle_rad))

    def translate(self, other_point):
        return Point(self.x + other_point.x, self.y + other_point.y)


class PolarPoint(Point):
    """
    Polar coordinate (radians, meters)
    """

    def __init__(self, angle, dist):
        self.angle = self.angle_rad = angle # radians
        self.dist = dist # distance in meters
        self.angle_deg = math.degrees(self.angle)
        self.x = self.dist * math.cos(self.angle_rad)
        self.y = self.dist * math.sin(self.angle_rad)


class ImageRecognition(object):

    GOAL_FIELD_DILATION = 50
    GOAL_BOTTOM = 200

    # Y U Y V
    FIELD_LOWER = 30, 0, 30, 0
    FIELD_UPPER = 255, 140, 255, 140

    YELLOW_LOWER = 128, 50, 128, 150
    YELLOW_UPPER = 255, 100, 255, 200


    BLUE_LOWER = 0, 140, 0, 0
    BLUE_UPPER = 255, 255, 255, 140


    BALL_LOWER = 64, 0, 64, 200
    BALL_UPPER = 255, 100, 255, 255


    KICKER_OFFSET = 0
    # Ball search scope vertically
    BALLS_BOTTOM = 300

    def __init__(self, frame, copy=True, camera_height=0.22, camera_mount_radius=0.07, dist_goals=4.6, camera_vert_fov=72, camera_horiz_fov=54):
        """
        Create image recognition object for 8-headed camera mount which internally
        tracks the state and corrects sensor readings

        Keyword arguments:
        frame -- 320x4230 YUYV frame
        dist_goals -- Goal to goal distance
        camera_height -- Camera height from the floor (m)
        camera_mount_radius -- Camera distance from the center of the robot (m)
        camera_vert_fov -- Camera field of view vertically (deg)
        camera_horiz_fov -- Camera field of view horizontally (deg)
        """

#        self.ball_grabbed_green1 = 584>>1, 4320- (2200 + 32) #4320-2184,
#        self.ball_grabbed_orange = 606>>1, 4320-(2200) # 4320-2216
#        self.ball_grabbed_green2 = 584>>1, 4320-(2200 - 32) #4320-2248,

        self.ball_grabbed_green1 = 564>>1, 4320-2184
        self.ball_grabbed_orange = 586>>1, 4320-2216
        self.ball_grabbed_green2 = 564>>1, 4320-2248,




        self.dist_goals = dist_goals
        self.camera_height = camera_height
        self.camera_mount_radius = camera_mount_radius
        self.camera_horiz_fov_rad = math.radians(camera_horiz_fov)
        self.camera_vert_fov_rad = math.radians(camera_vert_fov)
        self.update(frame)

        assert abs(self.x_to_deg(self.deg_to_x(50)) - 50) < 0.1, self.x_to_deg(self.deg_to_x(50))
        assert abs(self.y_to_dist(self.dist_to_y(2.0)) - 2.0) < 0.1

    def update(self, frame):
        assert frame.shape == (4320, 320, 4)
        self.frame = frame
        self.field_mask = self._recognize_field(self.FIELD_LOWER, self.FIELD_UPPER)

        self.goal_blue_mask, \
        self.goal_blue, \
        self.goal_blue_rect = self._recognize_goal(self.BLUE_LOWER, self.BLUE_UPPER)
        self.goal_yellow_mask, \
        self.goal_yellow,  \
        self.goal_yellow_rect = self._recognize_goal(self.YELLOW_LOWER, self.YELLOW_UPPER)

        self.robot, self.orientation = self._position_robot() # Calculate x and y coords on the field and angle to grid
        self.balls_mask, self.balls = self._recognize_balls()
        self.ball_grabbed = self._recognize_ball_grabbed()


    def _recognize_ball_grabbed(self):
        if self.frame is None:
            return False
        orange_pixels = cv2.inRange(self.frame[
            self.ball_grabbed_orange[1]-4:self.ball_grabbed_orange[1]+4,
            self.ball_grabbed_orange[0]-4:self.ball_grabbed_orange[0]+4],
            self.BALL_LOWER, self.BALL_UPPER).sum()
        green1_pixels = cv2.inRange(self.frame[
            self.ball_grabbed_green1[1]-4:self.ball_grabbed_green1[1]+4,
            self.ball_grabbed_green1[0]-4:self.ball_grabbed_green1[0]+4],
            self.FIELD_LOWER, self.FIELD_UPPER).sum()
        green2_pixels = cv2.inRange(self.frame[
            self.ball_grabbed_green2[1]-4:self.ball_grabbed_green2[1]+4,
            self.ball_grabbed_green2[0]-4:self.ball_grabbed_green2[0]+4],
            self.FIELD_LOWER, self.FIELD_UPPER).sum()
        #print("orange:", orange_pixels, "green1_pixels:", green1_pixels, "green2_pixels:", green2_pixels)
        return orange_pixels > 1000 and green1_pixels > 1000 and green2_pixels > 1000

    def _position_robot(self):
        return None, None

        if not self.goal_blue or not self.goal_yellow:
            logger.info("Both goal not detected!")
            return None, None

        if self.goal_blue.dist + self.goal_yellow.dist > 7:
            logger.info("Both goals overlap!")
            #print("GLITCH READING DISTANCES, got:", self.goal_blue.dist, self.goal_yellow.dist)
            return None, None

        # Perceived angle between goals
        rad_diff = abs(self.goal_yellow.angle_rad - self.goal_blue.angle_rad)

        # Correct perceived distances based on the angle and known goal-goal distance
        derived_dist = math.sqrt(self.goal_yellow.dist ** 2 + self.goal_blue.dist ** 2 - 2 * self.goal_yellow.dist * self.goal_blue.dist * math.cos(rad_diff))

        if not derived_dist:
            # FAILBOX
            return None, None


        correction_factor = self.dist_goals / derived_dist # Divide goal-goal disance with percevied distance
#        print("correction:", correction_factor)

        # TODO: This is stupid, never assign without purpose
        self.goal_yellow.dist *= correction_factor
        self.goal_blue.dist *= correction_factor

        #assert self.goal_blue.dist + self.goal_yellow.dist > self.dist_goals, "%.1fm" % (self.goal_blue.dist + self.goal_yellow.dist)
        assert self.dist_goals ** 2 - (self.goal_blue.dist ** 2 + self.goal_yellow.dist ** 2 - 2 * self.goal_blue.dist * self.goal_yellow.dist * math.cos(rad_diff)) < 0.00001, \
          "%.1f %.1f" % (self.dist_goals ** 2, self.goal_blue.dist ** 2 + self.goal_yellow.dist ** 2 - 2 * self.goal_blue.dist * self.goal_yellow.dist * math.cos(rad_diff))

        # Calculate distance projection along the line from goal to goal
        robot_x = (self.dist_goals**2-self.goal_blue.dist**2+self.goal_yellow.dist**2)/(2*self.dist_goals)
        try:
            robot_y = -math.sqrt(self.goal_yellow.dist**2-robot_x**2)
        except ValueError:
            logger.info("Triangulation failed")
            return None, None

        # thx Fred, Lauri's too st00pid for this shit
        if rad_diff > math.pi:
            robot_y = -robot_y
        if self.goal_yellow.angle_rad > self.goal_blue.angle_rad:
            robot_y = -robot_y

        # Knowing distance of goals and the angle between goals
        # we can derive the other angles
        sine = math.sin(rad_diff)
        if sine == 0:
            logger.info("Both goals overlap!")
            return None, None
        circumcircle_diameter = 4.6 / sine
        rad_blue = math.asin(self.goal_blue.dist / circumcircle_diameter)
        rad_yellow = math.asin(self.goal_yellow.dist / circumcircle_diameter)

        # TODO: Check if we got sensible triangle here

        orientation_rad = (-rad_blue -self.goal_blue.angle_rad) % (2*math.pi)

        return Point(robot_x, robot_y), orientation_rad


    def _recognize_field(self, lower, upper):
        mask = cv2.inRange(self.frame, lower, upper)
        assert mask.shape == (4320, 320)
        mask = cv2.erode(mask, None, iterations=4)

        slices = []

        # iterate over cameras because otherwise convex hull wraps around distorted field edges
        for j in range(0,9):
            sliced = mask[j*480:(j+1)*480,:]
            _, contours, hierarchy = cv2.findContours(sliced, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            sliced = mask[j*480:(j+1)*480,:]
            contours = [c for c in contours if cv2.contourArea(c) > 30]
            if contours:
                merged = np.vstack(contours) # merge contours
                hull = cv2.convexHull(merged) # get convex hull poly
                cv2.drawContours(sliced, [hull],0, 9, -1) # Fill in mask with convex hull
            slices.append(sliced)

        mask = np.vstack(slices)
        assert mask.shape == (4320, 320), "got instead %s" % repr(mask.shape)

        return mask

    def _recognize_goal(self, lower, upper):
        # Recognize goal
        mask = cv2.inRange(self.frame[:,:self.BALLS_BOTTOM-self.GOAL_FIELD_DILATION], lower, upper)
        mask = cv2.erode(mask, None, iterations=4)
        mask = cv2.bitwise_and(mask, self.field_mask[:,self.GOAL_FIELD_DILATION:self.BALLS_BOTTOM])

        rect = None
        maxwidth = 0
        step = 1 * 480
        scope = 3 * 480
        rects = []
        for j in range(0,9):
            sliced = mask[j*step:j*step+scope,:]

            _, contours, hierarchy = cv2.findContours(sliced, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours = sorted(contours, key=cv2.contourArea)[-1:]

            if contours:
                merged = np.vstack(contours)
                hull = cv2.convexHull(merged) # get convex hull poly
                y,x,h,w = cv2.boundingRect(hull)

                if w < scope and w > maxwidth:
                    maxwidth = w
                    rect = (x+j*step),2*y,w,h*2
                    rects.append(rect)

        if maxwidth:
            x,y,w,h = rect # done
            return mask, PolarPoint(self.x_to_rad(x+w/2.0), self.y_to_dist(y+h+self.GOAL_FIELD_DILATION)), rects
        return mask, None, []

    def _recognize_balls(self):
        """
        Return mask for balls and list of balls
        """
        mask = cv2.inRange(self.frame[:,:self.BALLS_BOTTOM], self.BALL_LOWER, self.BALL_UPPER)
        mask = cv2.bitwise_and(mask, self.field_mask[:,:self.BALLS_BOTTOM])
        cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        balls = set()
        skipped = set()
        
        for c in cnts:
            y, x, h, w = cv2.boundingRect(c)

            # Balls on the other side look tiny
            if w < 2 or h < 2:
                continue

            # Adjust for the fact that we have 320 YUYV pixels
            radius = w >> 1 if w > h else h >> 1
            cx = x + radius
            cy = y * 2 + radius

            relative = PolarPoint(self.x_to_rad(cx), self.y_to_dist(cy+radius))

            """
            # Skip balls in blue goal
            if self.goal_blue:
                bx, by, bw, bh = self.goal_blue_rect[0]
                if cx + radius > bx and cx - radius < bx+bw and cy+radius > by and cy-radius < by+bh:
                    continue

            # Skip balls in yellow goal
            if self.goal_yellow:
                yx, yy, yw, yh = self.goal_yellow_rect[0]
                if cx + radius > yx and cx - radius < yx+yw and cy+radius > yy and cy-radius < yy+yh:
                    continue
            """

            if self.robot and self.orientation:
                absolute = relative.rotate(-self.orientation).translate(self.robot)
            else:
                absolute = None
            ball_coords = relative, absolute, cx, cy, radius
            balls.add(ball_coords)
        #TODO: better alghoritm to sort balls
        #return mask, sorted(balls, key=lambda b:b[0].dist * abs( b[0].angle / 180) )
        return mask, sorted(balls, key=lambda b:b[0].dist)


    def dist_to_y(self, d):
        """
        Convert object distance to panorama image y coordinate
        """
        return int(640*math.atan2(self.camera_height, d - self.camera_mount_radius)/(0.2*2*math.pi))

    def y_to_dist(self, y):
        """
        Convert panorama image y coordinate to distance
        """
        return self.camera_height / math.tan(y * self.camera_vert_fov_rad / 640) + self.camera_mount_radius

    def deg_to_x(self, d):
        """
        Convert degrees from the kicker to panorama image x coordinate
        """
        d = d % 360
        if d > 180: d -= 360
        return int(d*3840/360+240+1920)-self.KICKER_OFFSET


    def x_to_deg(self, x):
        x += self.KICKER_OFFSET
        d = (x-2160.0)*360/(3840)
        return d % 360

    def x_to_rad(self, x):
        """
        Convert panorama image x coordinate to angle in radians from the center of the image
        (angle from the the kicker)
        """
        x += self.KICKER_OFFSET
        d = (x-2160.0)*(math.pi*2)/(3840)
        if d > math.pi:
            d -= math.pi * 2
        if d < -math.pi:
            d += math.pi * 2
        return d

import json

class ImageRecognizer(ManagedThread):
    def step(self, frame):
        assert frame.shape == (4320, 320, 4), "Got %s instead" % repr(frame.shape)
        r = ImageRecognition(frame)
        self.produce(r, self.grabber)

        # TODO: Put this in another sensible place, probably as a ImageRecognizer consumer
        if r.robot:
            for websocket in self.websockets:
                websocket.send(json.dumps(dict(
                    action="position-robot",
                    x=r.robot.x,
                    y=r.robot.y)))

