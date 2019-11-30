from time import time
from typing import Tuple

import cv2
import cv2.aruco as aruco
import numpy as np
import math
import logging

from shared import get_image_publisher
from .line_fit import goal_to_dist
from .managed_threading import ManagedThread, ThreadManager

logger = logging.getLogger("image_recognition")

aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()


class Point:
    """
    Cartesian coordinate (meters)
    """

    def __init__(self, x, y, suspicious=False):
        self.x = x
        self.y = y
        self.angle_rad = math.atan2(x, y)
        self.angle_deg = math.degrees(self.angle_rad)
        self.dist = (x ** 2 + y ** 2) ** 0.5
        self.suspicious = suspicious

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

    def __init__(self, angle_rad=None, dist=None, suspicious=False, radius=8, vx=0, vy=0, **_):
        self.angle_rad = angle_rad  # radians
        self.dist = dist  # distance in meters
        self.angle_deg = math.degrees(self.angle_rad)
        self.x = self.dist * math.cos(self.angle_rad)
        self.y = self.dist * math.sin(self.angle_rad)
        self.suspicious = suspicious
        self.radius = radius
        self.vx = vx
        self.vy = vy

    @property
    def angle_deg_abs(self):
        return abs(self.angle_deg)

    @property
    def angle_rad_abs(self):
        return abs(self.angle_rad)

    def serialize(self):
        return dict(
            angle_rad=self.angle_rad,
            dist=self.dist,
            angle_deg=self.angle_deg,
            x=self.x,
            y=self.y,
            suspicious=self.suspicious,
            radius=self.radius,
            vx=self.vx,
            vy=self.vy,
        )


class ImageRecognition:
    GOAL_FIELD_DILATION = 50
    GOAL_BOTTOM = 200
    # Ball search scope vertically
    BALLS_BOTTOM = 300

    def __init__(self, frame, camera_height=0.265, camera_mount_radius=0.07, dist_goals=4.6,
                 camera_vert_fov=72, camera_horiz_fov=54, camera_config=None, color_config=None):
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
        # unused currently
        self.robot = None
        self.orientation = None

        camera_config = camera_config or {}
        color_config = color_config or {}
        self.kicker_offset = camera_config.get('global', {}).get('kicker offset', 0)

        field = color_config.get("field", {})
        ball = color_config.get("ball", {})
        goal_A = color_config.get("goal A", {})
        goal_B = color_config.get("goal B", {})

        self.FIELD_LOWER, self.FIELD_UPPER = self.unpack(field)
        self.BALL_LOWER, self.BALL_UPPER = self.unpack(ball)
        self.BLUE_LOWER, self.BLUE_UPPER = self.unpack(goal_A)
        self.YELLOW_LOWER, self.YELLOW_UPPER = self.unpack(goal_B)

        self.dist_goals = dist_goals
        self.camera_height = camera_height
        self.camera_mount_radius = camera_mount_radius
        self.camera_horiz_fov_rad = math.radians(camera_horiz_fov)
        self.camera_vert_fov_rad = math.radians(camera_vert_fov)

        self.frame = frame
        # self.markers = self._recognize_markers()
        # print([ (id, round(dist))for id, dist in self.markers.items()])

        self.field_mask, self.field_contours = self._recognize_field(self.FIELD_LOWER, self.FIELD_UPPER)

        self.goal_blue_mask, self.goal_blue, self.goal_blue_rect, self.goal_blue_width_deg = \
            self._recognize_goal(self.BLUE_LOWER, self.BLUE_UPPER, ids=[10, 11])

        self.goal_yellow_mask, self.goal_yellow, self.goal_yellow_rect, self.goal_yellow_width_deg = \
            self._recognize_goal(self.YELLOW_LOWER, self.YELLOW_UPPER)

        # Calculate x and y coords on the field and angle to grid
        # self.robot, self.orientation = self._position_robot()

        self.balls_mask, self.balls = self._recognize_balls()
        self.closest_edge, self.field_center, \
        self.goal_angle_adjust, self.h_bigger, self.h_smaller = self._recognize_closest_edge()

        assert abs(self.x_to_deg(self.deg_to_x(50)) - 50) < 0.1, self.x_to_deg(self.deg_to_x(50))
        assert abs(self.y_to_dist(self.dist_to_y(2.0)) - 2.0) < 0.1

    def serialize(self):
        return dict(
            balls=[relative.serialize() for relative, absolute, cx, cy, radius in self.balls],
            goal_yellow=self.goal_yellow and self.goal_yellow.serialize(),
            goal_blue=self.goal_blue and self.goal_blue.serialize(),
            closest_edge=self.closest_edge and self.closest_edge.serialize(),
            goal_angle_adjust=[self.goal_angle_adjust, self.h_bigger, self.h_smaller],
            field_contours=[cv2.boundingRect(hull) for hull in self.field_contours],
            goal_yellow_rect=self.goal_yellow_rect,
            goal_blue_rect=self.goal_blue_rect,
        )

    @staticmethod
    def unpack(mapping):
        luma = mapping['luma']
        chroma_red = mapping['chroma red']
        chroma_blue = mapping['chroma blue']
        lower, upper = zip(luma, chroma_blue, luma, chroma_red)
        return lower, upper

    def _recognize_closest_edge(self) -> Tuple[PolarPoint, PolarPoint, float, int, int]:
        """
        Recognize angle and distance to closest edge and to center of field
        """
        closest_dist = 99999
        closest_angle = 0
        dx = 0
        dy = 0
        y_map = {}
        for index, hull in enumerate(self.field_contours[:8]):
            y, x, h, w = cv2.boundingRect(hull)
            y_map[index] = y * 2
            rotation = (index - 4) * (math.pi * 2 / 8.0)
            dist = self.y_to_dist(y * 2)
            if dist < closest_dist:
                closest_dist = dist
                closest_angle = rotation
            # print("rot:", rotation, "dist:", dist, "xy:", math.sin(rotation) * dist, math.cos(rotation) * dist)
            dx += math.sin(rotation) * dist
            dy += math.cos(rotation) * dist

        bigger, smaller, goal_angle_adjust = None, None, 0.0
        if 3 in y_map and 5 in y_map:
            left_side = y_map[3]
            right_side = y_map[5]
            bigger = max(left_side, right_side)
            smaller = min(left_side, right_side)
            factor = min(bigger / (smaller or 1), 4) - 1
            sign = -1 if left_side < right_side else 1
            goal_angle_adjust = sign * factor
            # logger.info('GOAL_ANGLE ADJUST: %s / %s -> %s', bigger, smaller, goal_angle_adjust)

        # else:
        #     logger.info('GOAL_ANGLE ADJUST: %s', str(y_map))

        return PolarPoint(closest_angle, closest_dist), Point(dx / 8.0, dy / 8.0), goal_angle_adjust, bigger, smaller

    def _position_robot(self):
        return None, None
        if not self.goal_blue or not self.goal_yellow:
            logger.info("Both goal not detected!")
            return None, None

        if self.goal_blue.dist + self.goal_yellow.dist > 7:
            logger.info("Both goals overlap!")
            return None, None

        # Perceived angle between goals
        rad_diff = abs(self.goal_yellow.angle_rad - self.goal_blue.angle_rad)

        # Correct perceived distances based on the angle and known goal-goal distance
        derived_dist = math.sqrt(
            self.goal_yellow.dist ** 2 + self.goal_blue.dist ** 2 - 2 * self.goal_yellow.dist * self.goal_blue.dist * math.cos(
                rad_diff))

        if not derived_dist:
            logger.error("Triangulation failed 1")
            return None, None

        correction_factor = self.dist_goals / derived_dist  # Divide goal-goal disance with percevied distance
        # TODO: This is stupid, never assign without purpose
        # self.goal_yellow.dist *= correction_factor
        # self.goal_blue.dist *= correction_factor

        # assert self.goal_blue.dist + self.goal_yellow.dist > self.dist_goals, "%.1fm" % (self.goal_blue.dist + self.goal_yellow.dist)
        # assert self.dist_goals ** 2 - (self.goal_blue.dist ** 2 + self.goal_yellow.dist ** 2 - 2 * self.goal_blue.dist * self.goal_yellow.dist * math.cos(rad_diff)) < 0.00001

        # Calculate distance projection along the line from goal to goal
        robot_x = (self.dist_goals ** 2 - self.goal_blue.dist ** 2 + self.goal_yellow.dist ** 2) / (2 * self.dist_goals)
        try:
            robot_y = -math.sqrt(self.goal_yellow.dist ** 2 - robot_x ** 2)
        except ValueError:
            logger.error("Triangulation failed 2")
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
        orientation_rad = (-rad_blue - self.goal_blue.angle_rad) % (2 * math.pi)
        return Point(robot_x, robot_y), orientation_rad

    def _recognize_field(self, lower, upper):
        slices = []
        hulls = []
        overlap = 20  # overlap between cameras

        mask = cv2.inRange(self.frame[:3840], lower, upper)
        mask = np.vstack([mask, mask[:overlap]])
        mask[:, :30] = 0
        mask[:, 300:] = 0
        mask[:, 250:300] = 1

        # iterate over cameras because otherwise convex hull wraps around distorted field edges
        # field edges are straight lines within single camera scope
        for j in range(0, 9):
            begin, end = j * 480 - overlap, (j + 1) * 480 + overlap
            begin2, end2 = overlap, -overlap
            if begin < 0:
                begin = 0
                begin2 = 0
            if end > 4320:
                end = 4320
                end2 = 480 + overlap
            roi = mask[begin:end, :]
            _, contours, hierarchy = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours = [c for c in contours if cv2.contourArea(c) > 30]
            if contours:
                merged = np.vstack(contours)  # merge contours
                hull = cv2.convexHull(merged)  # get convex hull poly
                cv2.drawContours(roi, [hull], 0, 9, -1)  # Fill in mask with convex hull
                hulls.append(hull)
            slices.append(roi[begin2:end2])
        mask = np.vstack(slices)
        mask = mask[:3840]

        assert mask.shape == (3840, 320), "got instead %s" % repr(mask.shape)

        return mask, hulls

    def _recognize_markers(self):
        markers = {}
        #
        # factor = 1
        # j = 4
        # d = 0
        # gray = self.frame.reshape(5529600)[::2].reshape((4320, 640, 1))
        # gray = gray[(j - d) * 480:(j + d) * 480 + 480:]
        # gray = np.rot90(gray, 3).copy()
        # gray = cv2.flip(gray, 1)
        #
        # # factor = 0.5
        # # gray = cv2.resize(gray, None, fx=factor, fy=factor, interpolation=cv2.INTER_NEAREST)
        #
        # corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict)
        # if ids is not None:
        #     for marker_ids, marker in zip(ids, corners):
        #         a, b, c, d = [corner for corner in marker[0]]
        #         vertical = sum(corner[1] for corner in marker[0]) / 4
        #         d1, d2 = a - d, b - c
        #         dist = lambda D: (D[0] ** 2 + D[1] ** 2) ** 0.5
        #         d1, d2 = dist(d1), dist(d2)
        #         height = (d1 + d2) / 2
        #         dist_a = height_to_dist(height / factor)
        #         dist_b = vertical_to_dist(vertical / factor)
        #         # print(dist_a, dist_b)
        #         # self.markers[marker_ids[0]] = max(dist_a, dist_b)
        #         self.markers[marker_ids[0]] = (dist_a  + dist_b) / 2
        #         self.markers[marker_ids[0]] = dist_b
        #     print(self.markers)
        return markers

    def _recognize_goal(self, lower, upper, overlap=4, ids=[]):
        # Recognize goal
        field = self.frame[:3840, :self.BALLS_BOTTOM - self.GOAL_FIELD_DILATION]
        mask = cv2.inRange(field, lower, upper)

        mask = cv2.erode(mask, None, iterations=4)
        mask = cv2.bitwise_and(mask, self.field_mask[:, self.GOAL_FIELD_DILATION:self.BALLS_BOTTOM])

        rect = None
        rects = []
        cnts = []

        # Iterate over cameras separately and generate mask for each camera
        for j in range(0, 8):
            roi = mask[j * 480:j * 480 + 480:]  # one camera
            _, contours, hierarchy = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours = [c for c in contours if cv2.contourArea(c) > 100]
            if contours:
                hull = cv2.convexHull(np.vstack(contours))
                y, x, h, w = cv2.boundingRect(hull)
                rect = (x + j * 480), 2 * y, w, h * 2
                rects.append(rect)
                cnts.append(hull)
            else:
                cnts.append(None)

        # Add two copies of contours to deal with goal having wider angle of three cameras in total
        for j in range(0, overlap):
            cnts.append(cnts[j])

        # Find widest contour spanning over three cameras
        maxwidth = 0
        for j in range(0, 11):
            # stack contours of three cameras
            contours = [c + (0, i * 480) for i, c in enumerate(cnts[j:j + 3]) if c is not None]
            if not contours:
                continue
            merged = np.vstack(contours)
            hull = cv2.convexHull(merged)  # get convex hull poly
            y, x, h, w = cv2.boundingRect(hull)
            if w > maxwidth:
                maxwidth = w
                rect = (x + j * 480), 2 * y, w, h * 2

        # print(time() - start, 'time')
        if maxwidth:
            x, y, w, h = rect  # done
            dist = goal_to_dist(y + h) / 100

            # markers = [dist for id, dist in self.markers.items() if id in ids]
            # if markers:
            #     dist = sum(markers) / len(markers) / 100
            return mask, PolarPoint(self.x_to_rad(x + w / 2.0) + math.radians(1), dist), rects, w * 360 / 3840.0
        return mask, None, [], 0

    def _recognize_balls(self):
        """
        Return mask for balls and list of balls
        """
        mask = cv2.inRange(self.frame[:3840, :self.BALLS_BOTTOM], self.BALL_LOWER, self.BALL_UPPER)
        mask = cv2.erode(mask, None, iterations=1)
        mask = cv2.bitwise_and(mask, self.field_mask[:3840, :self.BALLS_BOTTOM])
        mask = np.vstack([mask, mask[:480]])
        cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        balls = set()
        skipped = set()

        for c in cnts:
            y, x, h, w = cv2.boundingRect(c)

            # Balls on the other side look tiny
            if w < 2 or h < 2:
                continue

            # Adjust for the fact that we have 320 YUYV pixels
            diameter = w if w > h else (h << 1)
            radius = diameter >> 1
            cx = x + (w >> 1)
            cy = (y << 1) + h  # bottom half is really bad seen and reflects field color (h >> 1)

            # TODO: is this dependent on the size?
            relative = PolarPoint(self.x_to_rad(cx), self.y_to_dist(cy + radius), radius=radius, vx=cx, vy=cy)

            # TODO: most likely we dont need suspicious logic anymore, as balls are thrown in a basket.
            suspicious = False
            # SkipFlag balls in blue goal
            if self.goal_blue:
                bx, by, bw, bh = self.goal_blue_rect[0]
                if cx + radius > bx and cx - radius < bx + bw and cy + radius > by and cy - radius < by + bh:
                    suspicious = True

            # Flag balls in yellow goal
            if self.goal_yellow:
                yx, yy, yw, yh = self.goal_yellow_rect[0]
                if cx + radius > yx and cx - radius < yx + yw and cy + radius > yy and cy - radius < yy + yh:
                    suspicious = True
            relative.suspicious = suspicious

            if self.robot and self.orientation:
                absolute = relative.rotate(-self.orientation).translate(self.robot)
            else:
                absolute = None
            ball_coords = relative, absolute, cx, cy, radius
            balls.add(ball_coords)

        # TODO: better alghoritm to sort balls
        # return mask, sorted(
        #     balls,
        #     key=lambda b: b[0].dist + abs(b[0].angle_deg / 180) / 10,
        # )

        return mask, sorted(balls, key=lambda b: b[0].dist)

    def dist_to_y(self, d):
        """
        Convert object distance to panorama image y coordinate
        """
        return int(640 * math.atan2(self.camera_height, d - self.camera_mount_radius) / (0.2 * 2 * math.pi))

    def y_to_dist(self, y):
        """
        Convert panorama image y coordinate to distance
        """
        j = math.tan(y * self.camera_vert_fov_rad / 640)
        if j != 0:
            return self.camera_height / j + self.camera_mount_radius
        return 99999999  # infinity to prevent division by zero

    def deg_to_x(self, d, offset=None):
        """
        Convert degrees from the kicker to panorama image x coordinate
        """
        d = d % 360
        if d > 180: d -= 360
        return int(d * 3840 / 360) + self.kicker_offset

    def x_to_deg(self, x):
        d = (x - self.kicker_offset) * 360 / (3840)
        return d % 360

    def x_to_rad(self, x):
        """
        Convert panorama image x coordinate to angle in radians from the center of the image
        (angle from the the kicker)
        """
        d = (x - self.kicker_offset) * (math.pi * 2) / (3840)
        if d > math.pi:
            d -= math.pi * 2
        if d < -math.pi:
            d += math.pi * 2
        return d


class ImageRecognizer(ManagedThread):
    last_frame = None

    def __init__(self, upstream_producer=None, framedrop=0, lossy=True, config_manager=None, publisher=None):
        super().__init__(upstream_producer, framedrop, lossy)
        self.camera_config = {}
        self.color_config = {}
        self.config_manager = config_manager
        self.publisher = publisher
        self.counter = 0
        self.refresh_config()
        self.roundtrip_start = time()
        self.silent = False

        self.br_color = None
        self.br_field_mask = None
        self.br_balls_mask = None
        self.br_goal_blue_mask = None
        self.br_goal_yellow_mask = None

    def broadcast(self, color: np.ndarray, field_mask: np.ndarray, balls_mask: np.ndarray, goal_blue_mask: np.ndarray,
                  goal_yellow_mask: np.ndarray):
        if self.br_color is None:
            self.br_color = get_image_publisher("shm://recognizer-color", color.shape, color.dtype)
            self.br_field_mask = get_image_publisher("shm://recognizer-field_mask", field_mask.shape, field_mask.dtype)
            self.br_balls_mask = get_image_publisher("shm://recognizer-balls_mask", balls_mask.shape, balls_mask.dtype)
            self.br_goal_blue_mask = get_image_publisher("shm://recognizer-goal_blue_mask", goal_blue_mask.shape,
                                                         goal_blue_mask.dtype)
            self.br_goal_yellow_mask = get_image_publisher("shm://recognizer-goal_yellow_mask", goal_yellow_mask.shape,
                                                           goal_yellow_mask.dtype)

        self.br_color[:, :, :] = color
        self.br_field_mask[:, :] = field_mask
        self.br_balls_mask[:, :] = balls_mask
        self.br_goal_blue_mask[:, :] = goal_blue_mask
        self.br_goal_yellow_mask[:, :] = goal_yellow_mask

    def log_roundtrip(self):
        roundtrip = 1 / (time() - self.roundtrip_start)
        self.roundtrip_start = time()
        if not self.silent:
            self.publisher.logger.info_throttle(
                2,
                "fps:%.0f lat:%.2f roundtrip:%.2f" % (self.average_fps or 0, self.average_latency or 0, roundtrip))

    def refresh_config(self, *_):
        logger.info("settings update received")
        if self.config_manager:
            self.camera_config = self.config_manager.get_value('camera')
            self.color_config = self.config_manager.get_value('color')

    def step(self, frame):
        r = ImageRecognition(
            frame,
            camera_config=self.camera_config,
            color_config=self.color_config,
        )

        self.counter += 1
        self.broadcast(r.frame, r.field_mask, r.balls_mask, r.goal_blue_mask, r.goal_yellow_mask)

        if self.publisher:
            serialized = dict(**r.serialize(), fps=self.average_fps, lat=self.average_latency)
            self.publisher.command(**serialized)

        self.log_roundtrip()


class Player(ManagedThread):
    """
    Player thread reads RGB frames from a video file and converts them to YUYV frames
    suitable for ImageRecognizer thread
    """

    def on_enabled(self, filename):
        self.cap = cv2.VideoCapture(filename)

    def step(self, filename):
        succ, frame = self.cap.read()
        if not succ:
            self.disable()
            return
        rotated = np.rot90(frame, 1).copy()
        yuv = cv2.cvtColor(rotated[:3840], cv2.COLOR_RGB2YCR_CB)
        y, u, v = np.dsplit(yuv, 3)
        y = y[:, ::2]
        u = u[:, ::2]
        v = v[:, ::2]
        yuyv = np.dstack([y, u, y, v])
        self.produce(yuyv)


class Shower(ManagedThread):
    def step(self, buf, resized, frame, r):
        frame = frame[:, :3840]
        height, width, _ = frame.shape
        # Align kicker to the middle of first row

        x1 = r.kicker_offset - width / 4
        x2 = r.kicker_offset + width / 4
        j = np.hstack([frame[:, x2:], frame[:, :x1]])
        frame = np.vstack([frame[:, x1:x2], j])

        frame = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)
        cv2.imshow("Got balls?", frame)
        if cv2.waitKey(1) >= 0:
            self.disable()
            self.stop()


if __name__ == "__main__":
    """
    Apply image recognition on a recorded videofile, usage:
    python3 image_recognition.py filename.avi
    """
    from visualization import Visualizer

    import sys

    path = sys.argv[1]
    player = Player(path)

    image_recognizer = ImageRecognizer(player)
    visualizer = Visualizer(image_recognizer)
    shower = Shower(visualizer)

    image_recognizer.grabber = player
    player.start()
    image_recognizer.start()
    visualizer.start()
    shower.start()

    manager = ThreadManager()
    manager.register(player)
    manager.register(image_recognizer)
    manager.register(visualizer)
    manager.register(shower)
    manager.start()

    player.enable()
    image_recognizer.enable()
    visualizer.enable()
    shower.enable()

    # Wait
    shower.join()
    image_recognizer.stop()
    visualizer.stop()
    player.stop()
