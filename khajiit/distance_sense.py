import threading
from time import sleep, time

import pyrealsense2 as rs
from pyrealsense2.pyrealsense2 import STDepthTableControl, depth_sensor as DepthSensor

import numpy as np
import cv2 as cv

from shared import get_image_publisher, ImageSubscriber

# https://github.com/IntelRealSense/librealsense/issues/3139

class StreamingMovingAverage:
    def __init__(self, window_size):
        self.window_size = window_size
        self.values = []
        self.sum = 0

    def __call__(self, value):
        if np.isnan(value):
            return value
        self.values.append(value)
        self.sum += value
        if len(self.values) > self.window_size:
            self.sum -= self.values.pop(0)
        return float(self.sum) / len(self.values)


class Distance:
    constants = [0.2215416616277008, 0.8260044802493705, -20.362320905330222]
    poly = lambda x, a, b, e: a * x ** b + e

    @classmethod
    def transform(cls, x: float) -> float:
        return cls.poly(x, *cls.constants)


class CaptureDistance:

    def __init__(self, lower=(96, 59, 53, ), upper=(160, 255, 130, ), callback=None) -> None:
        self.lower, self.upper = lower, upper
        self.callback = callback
        self.pipeline = rs.pipeline()
        config = rs.config()

        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # set depth units, this should give us better resolution sub 5m?
        device = rs.context().query_devices()[0]
        advnc_mode = rs.rs400_advanced_mode(device)
        depth_table_control_group: STDepthTableControl = advnc_mode.get_depth_table()
        depth_table_control_group.depthUnits = 500
        advnc_mode.set_depth_table(depth_table_control_group)

        # Start streaming
        profile = self.pipeline.start(config)

        # Getting the depth sensor's depth scale (see rs-align example for explanation)
        depth_sensor: DepthSensor = profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        print("Depth Scale is: ", depth_scale)

        align_to = rs.stream.color
        self.align = rs.align(align_to)

        self.pipeline.wait_for_frames()
        sleep(1)

        sensors = sum([dev.query_sensors() for dev in rs.context().query_devices()], [])
        color_sensor = next(sensor for sensor in sensors if sensor.get_info(rs.camera_info.name) == "RGB Camera")

        print("Setting RGB camera sensor settings")
        # exposure: 166.0
        # white balance: 4600.0
        # gain: 64.0
        # auto exposure enabled: 1.0
        # exposure: 166.0
        # white balance: 4600.0
        # gain: 64.0
        color_sensor.set_option(rs.option.saturation, 60)
        color_sensor.set_option(rs.option.exposure, 166)
        color_sensor.set_option(rs.option.enable_auto_exposure, 0)
        color_sensor.set_option(rs.option.white_balance, 4600)
        color_sensor.set_option(rs.option.enable_auto_white_balance, 0)
        color_sensor.set_option(rs.option.gain, 64)
        align = rs.align(align_to)

        # color_sensor.set_option(rs.option.enable_auto_exposure, 0)
        print("auto exposure enabled: {}".format(color_sensor.get_option(rs.option.enable_auto_exposure)))
        print("exposure: {}".format(color_sensor.get_option(rs.option.exposure)))  # 166
        print("white balance: {}".format(color_sensor.get_option(rs.option.white_balance)))
        print("gain: {}".format(color_sensor.get_option(rs.option.gain)))  # 64

        self.average = StreamingMovingAverage(20)
        self.average_raw = StreamingMovingAverage(20)

        self.average_fps = StreamingMovingAverage(20)
        self.average_area = StreamingMovingAverage(20)

        self.color = None
        self.distance = None
        self.fps = 0
        self.area = 0

        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def broadcast(self, color: np.ndarray):
        if self.color is None:
            self.color = get_image_publisher("shm://depth-color", color.shape, np.uint8)
        self.color[:, :, :] = color

    def run(self):
        try:
            start = time()
            while True:
                frames = self.pipeline.wait_for_frames()
                aligned_frames = self.align.process(frames)
                aligned_depth_frame = aligned_frames.get_depth_frame()  # aligned_depth_frame is a 640x480 depth image
                color_frame = aligned_frames.get_color_frame()

                if not aligned_depth_frame or not color_frame:
                    continue

                depth_image = np.asanyarray(aligned_depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())

                # color_image = cv.medianBlur(color_image, 11)
                color_image = cv.GaussianBlur(color_image, (3, 3), 0)
                hsv = cv.cvtColor(color_image, cv.COLOR_BGR2HSV)

                lower_range = np.array(self.lower)
                upper_range = np.array(self.upper)

                mask = cv.inRange(hsv, lower_range, upper_range)

                cut = mask.copy()
                cut[:, :] = 0
                cut[180:, :] = 1

                depth = depth_image.astype(np.double)
                nonzero = depth != 0
                full_mask = mask & cut & nonzero

                kernel = np.ones((5, 5), np.uint8)
                full_mask = cv.morphologyEx(full_mask, cv.MORPH_OPEN, kernel)

                depth_cut = cv.bitwise_and(depth, depth, mask=full_mask)

                area = np.sum(full_mask)
                distance_raw = np.sum(depth_cut) / area
                distance_raw = self.average_raw(distance_raw) or 1

                self.area = self.average_area(area)
                self.distance = Distance.transform(distance_raw)

                bg_removed = cv.bitwise_and(color_image, color_image, mask=full_mask)
                color_masked = cv.bitwise_and(color_image, color_image, mask=mask)
                bg_removed = cv.putText(bg_removed, f"{self.distance:.2f} - {distance_raw:.0f}", (50, 50),
                                        cv.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 2)

                # Render images
                depth_colormap = cv.applyColorMap(cv.convertScaleAbs(depth_image, alpha=0.03), cv.COLORMAP_JET)

                images = np.hstack((color_masked, bg_removed, depth_colormap))

                self.broadcast(images)
                self.fps = self.average_fps(1 / (time() - start))
                start = time()
        finally:
            self.pipeline.stop()


if __name__ == '__main__':
    capture = CaptureDistance((96, 59, 53, ),(160, 255, 130, ))
    capture.run()
