import logging
from threading import Thread
from time import sleep
from typing import Tuple, List, Optional

import numpy as np
import cv2 as cv

from shared import attach
from utils import RecognitionState
from .line_fit import dist_to_rpm

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
        self.jpeg = None

        self.thread = Thread(target=self.run, daemon=True)

    def run(self):
        shared: np.ndarray = attach("shm://recognizer-color")

        while True:
            sleep(0.015)
            if not self.recognition:
                continue

            rec = self.recognition

            converted = np.swapaxes(cv.cvtColor(shared.reshape((4320, 640, 2)), cv.COLOR_YUV2BGR_YUYV), 0, 1)
            frame = converted.copy()  # This speeds up whole lot

            # Kicker offset
            cv.line(frame, (self.kicker_offset, frame.shape[0] - 50), (self.kicker_offset, frame.shape[0]), (255, 255, 255), 3)

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
            for relative in rec.balls:
                x, y = relative.x, relative.y

                cv.circle(frame, (x, y), radius, (255, 255, 255) if index else (0, 0, 255), 3)

                x = rec.deg_to_x(relative.angle_deg)

                cv.putText(
                    frame, "%.1fdeg" % relative.angle_deg, (x + 20, y - 20), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 4)
                cv.putText(
                    frame, "%.2fm" % relative.dist, (x + 20, y + 40), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 4)
                index += 1

                if index < 2:
                    point = x, y
                    if prev is not None:
                        cv.line(frame, prev, point, (0, 0, 255), 4)
                    prev = point

            # Visualize goals
            if rec.goal_yellow:
                for delta in -3840, 0, 3840:
                    x = rec.deg_to_x(rec.goal_yellow.angle_deg) + delta
                    cv.line(frame, (x, 0), (x, rec.GOAL_BOTTOM - 120), (255, 255, 255), 3)
                    cv.putText(frame, "%.1fdeg" % rec.goal_yellow.angle_deg, (x + 90, rec.GOAL_BOTTOM + 120),
                                cv.FONT_HERSHEY_SIMPLEX, 3, (0, 0, 0), 8)
                    cv.putText(frame, "%.2fm" % rec.goal_yellow.dist, (x, rec.GOAL_BOTTOM - 30), cv.FONT_HERSHEY_SIMPLEX, 2,
                                (0, 0, 0), 4)

                for rect in rec.goal_yellow_rect:
                    cv.rectangle(frame, (rect[0], rect[1]), (rect[2] + rect[0], rect[3] + rect[1]), (32, 32, 255), 16)

            if rec.goal_blue:
                for delta in -3840, 0, 3840:
                    x = rec.deg_to_x(rec.goal_blue.angle_deg) + delta
                    cv.line(frame, (x, 0), (x, rec.GOAL_BOTTOM - 120), (255, 255, 255), 3)
                    cv.putText(frame, "%.1fdeg" % rec.goal_blue.angle_deg, (x + 90, rec.GOAL_BOTTOM + 120),
                                cv.FONT_HERSHEY_SIMPLEX, 3, (0, 0, 0), 8)
                    cv.putText(frame, "%.2fm" % rec.goal_blue.dist, (x, rec.GOAL_BOTTOM - 30), cv.FONT_HERSHEY_SIMPLEX, 2,
                                (0, 0, 0), 4)

                for rect in rec.goal_blue_rect:
                    cv.rectangle(frame, (rect[0], rect[1]), (rect[2] + rect[0], rect[3] + rect[1]), (255, 32, 32), 16)

            target_goal = rec.goal_blue
            if target_goal:
                # dist = target_goal.dist * 100
                # pwm = dist_to_rpm(dist)
                # angle = rec.goal_blue.angle_deg
                # y = sum(y + h for x, y, w, h in rec.goal_blue_rect) / len(rec.goal_blue_rect)
                # dist_str = "DIST %.0f" % dist
                # if rec.goal_yellow:
                #     dist_str = "DIST B%.0f P%.0f" % (dist, rec.goal_yellow.dist * 100)
                # cv.putText(frame, dist_str, (50, 100), cv.FONT_HERSHEY_SIMPLEX, 3, (0, 0, 0), 12)
                # cv.putText(frame, "PWM %.0f" % pwm, (50, 200), cv.FONT_HERSHEY_SIMPLEX, 3, (0, 0, 0), 12)
                # cv.putText(frame, "ANG %.0f" % angle, (50, 300), cv.FONT_HERSHEY_SIMPLEX, 3, (0, 0, 0), 12)
                # cv.putText(frame, "Y %.0f" % y, (50, 400), cv.FONT_HERSHEY_SIMPLEX, 3, (0, 0, 0), 12)
                cv.putText(frame, "ADJ %.2f" % rec.goal_angle_adjust, (50, 500), cv.FONT_HERSHEY_SIMPLEX, 3, (0, 0, 0), 12)

            # if rec.goal_yellow and rec.goal_blue:
            #     logger.info(f"goal_yellow: %.0f goal_blue: %.0f", rec.goal_yellow.dist * 100, rec.goal_blue.dist * 100)

            if not self.DEBUG_MASK or self.type_str == 'VIDEO':  # TODO: Read from config manager
                resized = cv.resize(frame, (0, 0), fx=self.ZOOM, fy=self.ZOOM)
                ret, jpeg = cv.imencode('.jpg', frame, (cv.IMWRITE_JPEG_QUALITY, 50))
                buf = jpeg.tostring()
                resized = frame
                self.produce(buf, resized, frame, rec)
                return

            # Visualize field mask
            field_mask = np.swapaxes(np.repeat(rec.field_mask, 2, axis=1), 0, 1)
            field_mask = np.hstack([field_mask, field_mask[:, :480]])

            logger.info(f"converted: {converted.shape} mask {field_mask.shape}")

            field_cutout = cv.bitwise_and(converted, converted, mask=field_mask)
            cv.line(field_cutout, (0, 0), (5000, 0), (255, 255, 255), 2)
            cv.putText(field_cutout, "field detection", (80, field_cutout.shape[0] >> 1), cv.FONT_HERSHEY_SIMPLEX, 1,
                        (255, 255, 255), 2)

            # Visualize yellow mask
            sliced = converted[:rec.BALLS_BOTTOM * 2 - rec.GOAL_FIELD_DILATION * 2, :]
            goal_yellow_mask = np.swapaxes(np.repeat(rec.goal_yellow_mask, 2, axis=1), 0, 1)
            goal_yellow_mask = np.hstack([goal_yellow_mask, goal_yellow_mask[:, :480]])
            goal_yellow_cutout = cv.bitwise_and(sliced, sliced, mask=goal_yellow_mask)
            cv.line(goal_yellow_cutout, (0, 0), (5000, 0), (255, 255, 255), 2)
            cv.putText(goal_yellow_cutout, "yellow goal detection", (80, goal_yellow_cutout.shape[0] >> 1),
                        cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

            # Visualize blue mask
            sliced = converted[:rec.BALLS_BOTTOM * 2 - rec.GOAL_FIELD_DILATION * 2, :]
            goal_blue_mask = np.swapaxes(np.repeat(rec.goal_blue_mask, 2, axis=1), 0, 1)
            goal_blue_mask = np.hstack([goal_blue_mask, goal_blue_mask[:, :480]])
            goal_blue_cutout = cv.bitwise_and(sliced, sliced, mask=goal_blue_mask)
            cv.line(goal_blue_cutout, (0, 0), (5000, 0), (255, 255, 255), 2)
            cv.putText(goal_blue_cutout, "blue goal detection", (80, goal_blue_cutout.shape[0] >> 1),
                        cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

            # Visualize orange balls
            sliced = converted[:rec.BALLS_BOTTOM * 2]
            balls_mask = np.swapaxes(np.repeat(rec.balls_mask, 2, axis=1), 0, 1)
            # balls_mask = np.hstack([balls_mask, balls_mask[:,:480]])
            balls_cutout = cv.bitwise_and(sliced, sliced, mask=balls_mask)
            cv.line(balls_cutout, (0, 0), (5000, 0), (255, 255, 255), 2)
            cv.putText(balls_cutout, "balls detection", (80, balls_cutout.shape[0] >> 1), cv.FONT_HERSHEY_SIMPLEX, 1,
                        (255, 255, 255), 2)

            prev = None
            for relative, absolute, x, y, radius in rec.balls:
                cv.circle(balls_cutout, (x, y), radius, (255, 255, 255) if index else (0, 0, 255), 3)

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
