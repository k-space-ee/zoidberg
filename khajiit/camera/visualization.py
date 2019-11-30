import logging
from threading import Thread
from time import sleep
from typing import Tuple, List, Optional

import numpy as np
import cv2 as cv

from camera.image_recognition import ImageRecognition
from shared import attach
from utils import RecognitionState

logger = logging.getLogger('visualization')


def left_top(args):
    y, x = args
    return x - 4, y * 2 - 4


def right_bottom(args):
    y, x = args
    return x + 4, y * 2 + 4


class Visualizer:
    ZOOM = 0.5  # 0.2
    DEBUG_MASK = True
    type_str = 'VIDEO'

    def __init__(self, camera_config):
        self.kicker_offset = camera_config.get('global', {}).get('kicker offset', 0)
        self.recognition: Optional[RecognitionState] = None
        self.gamestate: dict = {}
        self.jpeg = None

        self.thread = Thread(target=self.run, daemon=True)

    def deg_to_x(self, d):
        """
        Convert degrees from the kicker to panorama image x coordinate
        """
        d = d % 360
        if d > 180: d -= 360
        return int(d * 3840 / 360) + self.kicker_offset

    def run(self):
        shared: np.ndarray = attach("shm://recognizer-color")
        br_field_mask: np.ndarray = attach("shm://recognizer-field_mask")
        br_balls_mask: np.ndarray = attach("shm://recognizer-balls_mask")
        br_goal_blue_mask: np.ndarray = attach("shm://recognizer-goal_blue_mask")
        br_goal_yellow_mask: np.ndarray = attach("shm://recognizer-goal_yellow_mask")

        const = ImageRecognition

        while True:
            sleep(0.015)
            if not self.recognition:
                continue

            rec = self.recognition

            converted = np.swapaxes(cv.cvtColor(shared.reshape((4320, 640, 2)), cv.COLOR_YUV2BGR_YUYV), 0, 1)
            frame = converted.copy()  # This speeds up whole lot

            # Kicker offset
            cv.line(frame, (self.kicker_offset, frame.shape[0] - 50), (self.kicker_offset, frame.shape[0]),
                    (255, 255, 255), 3)

            # Visualize field edges
            points: List[Tuple[int, int]] = []
            for index, (y, x, h, w) in enumerate(rec.field_contours):
                points.append((int(x + index * 480 + w / 2), 2 * y))

            if points:
                points = [(points[-1][0] - 3840, points[-1][1])] + points
                prev = None
                for i, point in enumerate(points):
                    color = (128, 255, 128)
                    if i in (4, 5, 6):
                        color = (255, 0, 255)
                    cv.putText(frame, "%dy" % point[1], point, cv.FONT_HERSHEY_SIMPLEX, 2, color, 4)

                    if prev is not None:
                        cv.line(frame, prev, point, (128, 255, 128), 4)

                    prev = point

            # Visualize balls
            index = 0
            prev = (self.kicker_offset, 640)
            for ball in rec.balls:
                x, y = int(ball.vx), int(ball.vy)

                cv.circle(frame, (int(x), int(y)), int(ball.radius), (255, 255, 255) if index else (0, 0, 255), 3)

                x = self.deg_to_x(ball.angle_deg)

                # cv.putText(
                #     frame, "%.1fdeg" % ball.angle_deg, (x + 20, y - 20), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 4)
                # cv.putText(
                #     frame, "%.2fm" % ball.dist, (x + 20, y + 40), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 4)
                index += 1

                if index < 3:
                    point = x, y
                    if prev is not None:
                        cv.line(frame, prev, point, (0, 0, 255), 4)
                    prev = point

            for ball in self.gamestate.get("id_balls", []):
                x, y = point = int(ball.get('vx', 0)), int(ball.get('vy', 0))
                radius = ball.get('radius', 8)
                id = ball.get('id', '')[:5]
                alive = ball.get('alive', 0)

                cv.circle(frame, point, radius, (255, 0, 255), 3)
                cv.putText(frame, f"{id}-{alive:.1f}", (x + 20, y - 20), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 4)

            closest_ball: Optional[dict] = self.gamestate.get("closest_ball")
            if closest_ball:
                prev = (self.kicker_offset, 640)
                x, y = point = int(closest_ball.get('vx', 0)), int(closest_ball.get('vy', 0))
                cv.circle(frame, point, closest_ball.get('radius', 8), (0, 255, 0), 8)
                cv.line(frame, prev, point, (0, 255, 0), 4)
                cv.putText(frame, f"~{closest_ball.get('id')}~", (x + 20, y - 20), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 4)

            # Visualize goals
            if rec.goal_yellow:
                for delta in -3840, 0, 3840:
                    x = self.deg_to_x(rec.goal_yellow.angle_deg) + delta
                    cv.line(frame, (x, 0), (x, const.GOAL_BOTTOM - 120), (255, 255, 255), 3)
                    cv.putText(frame, "%.1fdeg" % rec.goal_yellow.angle_deg, (x + 90, const.GOAL_BOTTOM + 120),
                               cv.FONT_HERSHEY_SIMPLEX, 3, (0, 0, 0), 8)
                    cv.putText(frame, "%.2fm" % rec.goal_yellow.dist, (x, const.GOAL_BOTTOM - 30),
                               cv.FONT_HERSHEY_SIMPLEX, 2,
                               (0, 0, 0), 4)

                for rect in rec.goal_yellow_rect:
                    cv.rectangle(frame, (rect[0], rect[1]), (rect[2] + rect[0], rect[3] + rect[1]), (32, 32, 255), 8)

            if rec.goal_blue:
                for delta in -3840, 0, 3840:
                    x = self.deg_to_x(rec.goal_blue.angle_deg) + delta
                    cv.line(frame, (x, 0), (x, const.GOAL_BOTTOM - 120), (255, 255, 255), 3)
                    cv.putText(frame, "%.2fdeg" % rec.goal_blue.angle_deg, (x + 90, const.GOAL_BOTTOM + 120),
                               cv.FONT_HERSHEY_SIMPLEX, 3, (0, 0, 0), 8)
                    cv.putText(frame, "%.2fm" % rec.goal_blue.dist, (x, const.GOAL_BOTTOM - 30),
                               cv.FONT_HERSHEY_SIMPLEX, 2,
                               (0, 0, 0), 4)

                for rect in rec.goal_blue_rect:
                    cv.rectangle(frame, (rect[0], rect[1]), (rect[2] + rect[0], rect[3] + rect[1]), (255, 32, 32), 8)

            dist = self.gamestate.get('dist')
            real_distance = self.gamestate.get('real_distance') or 0

            pwm = self.gamestate.get('pwm')
            angle = self.gamestate.get('angle')
            angle_adj = rec.angle_adjust

            if dist is not None:
                cv.putText(frame, f"DIST {dist:.0f} {real_distance:.0f}", (50, 100), cv.FONT_HERSHEY_SIMPLEX, 3, (0, 0, 0), 12)
            if pwm is not None:
                cv.putText(frame, f"PWM {pwm:.0f}", (50, 200), cv.FONT_HERSHEY_SIMPLEX, 3, (0, 0, 0), 12)
            if angle is not None:
                cv.putText(frame, f"ANG {angle:.1f}", (50, 300), cv.FONT_HERSHEY_SIMPLEX, 3, (0, 0, 0), 12)
            if angle_adj is not None:
                cv.putText(frame, f"ADJ {angle_adj:.1f}", (50, 500), cv.FONT_HERSHEY_SIMPLEX, 3, (0, 0, 0), 12)

            if not self.DEBUG_MASK or self.type_str == 'VIDEO':  # TODO: Read from config manager
                resized = cv.resize(frame, (0, 0), fx=self.ZOOM, fy=self.ZOOM)
                ret, jpeg = cv.imencode('.jpg', frame, (cv.IMWRITE_JPEG_QUALITY, 50))
                buf = jpeg.tostring()
                self.jpeg = buf
                continue

            # Visualize field mask
            field_mask = np.swapaxes(np.repeat(br_field_mask, 2, axis=1), 0, 1)
            field_mask = np.hstack([field_mask, field_mask[:, :480]])

            # logger.info(f"converted: {converted.shape} mask {field_mask.shape}")

            field_cutout = cv.bitwise_and(converted, converted, mask=field_mask)
            cv.line(field_cutout, (0, 0), (5000, 0), (255, 255, 255), 2)
            cv.putText(field_cutout, "field detection", (80, field_cutout.shape[0] >> 1), cv.FONT_HERSHEY_SIMPLEX, 1,
                       (255, 255, 255), 2)

            # Visualize yellow mask
            sliced = converted[:const.BALLS_BOTTOM * 2 - const.GOAL_FIELD_DILATION * 2, :]
            goal_yellow_mask = np.swapaxes(np.repeat(br_goal_yellow_mask, 2, axis=1), 0, 1)
            goal_yellow_mask = np.hstack([goal_yellow_mask, goal_yellow_mask[:, :480]])
            # goal_yellow_cutout = cv.bitwise_and(sliced, sliced, mask=goal_yellow_mask)
            goal_yellow_cutout = cv.cvtColor(goal_yellow_mask, cv.COLOR_GRAY2BGR) * 255
            goal_yellow_cutout = cv.cvtColor(goal_yellow_mask, cv.COLOR_GRAY2BGR) * 255

            cv.line(goal_yellow_cutout, (0, 0), (5000, 0), (255, 255, 255), 2)
            cv.putText(goal_yellow_cutout, "yellow goal detection", (80, goal_yellow_cutout.shape[0] >> 1),
                       cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

            # Visualize blue mask
            sliced = converted[:const.BALLS_BOTTOM * 2 - const.GOAL_FIELD_DILATION * 2, :]
            goal_blue_mask = np.swapaxes(np.repeat(br_goal_blue_mask, 2, axis=1), 0, 1)
            goal_blue_mask = np.hstack([goal_blue_mask, goal_blue_mask[:, :480]])
            # goal_blue_cutout = cv.bitwise_and(sliced, sliced, mask=goal_blue_mask)
            goal_blue_cutout = cv.cvtColor(goal_blue_mask, cv.COLOR_GRAY2BGR) * 255

            cv.line(goal_blue_cutout, (0, 0), (5000, 0), (255, 255, 255), 2)
            cv.putText(goal_blue_cutout, "blue goal detection", (80, goal_blue_cutout.shape[0] >> 1),
                       cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

            # Visualize orange balls
            sliced = converted[:const.BALLS_BOTTOM * 2]
            balls_mask = np.swapaxes(np.repeat(br_balls_mask, 2, axis=1), 0, 1)
            balls_cutout = cv.cvtColor(balls_mask, cv.COLOR_GRAY2BGR) * 255

            # cv.line(balls_cutout, (0, 0), (5000, 0), (255, 255, 255), 2)
            cv.putText(balls_cutout, "balls detection", (80, 50), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

            prev = None
            # for ball in rec.balls:
            #     cv.circle(balls_cutout, (int(ball.vx), int(ball.vy)), int(ball.radius), (255, 255, 255) if index else (0, 0, 255), 3)

            frame = np.vstack([
                frame,
                field_cutout,
                goal_blue_cutout,
                goal_yellow_cutout,
                balls_cutout,
            ])

            resized = cv.resize(frame, (0, 0), fx=self.ZOOM, fy=self.ZOOM)
            ret, jpeg = cv.imencode('.jpg', resized, (cv.IMWRITE_JPEG_QUALITY, 80))  # 60
            buf = jpeg.tostring()
            self.jpeg = buf
