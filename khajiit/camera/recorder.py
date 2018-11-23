import cv2
import logging
import os
from .managed_threading import ManagedThread
from datetime import datetime
import numpy as np

logger = logging.getLogger("recorder")


class Recorder(ManagedThread):
    def on_enabled(self, first_frame, *args):
        self.then = datetime.now()
        self.path = os.path.expanduser("~/bot-%s.avi") % datetime.now().strftime("%Y%m%d%H%M%S")
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.writer = cv2.VideoWriter(self.path, fourcc, 30, (4320, 640))
        logger.info("Recorder saving video to %s", self.path)

    def step(self, frame, *args):
        frame = np.rot90(cv2.cvtColor(frame.reshape((4320, 640, 2)), cv2.COLOR_YUV2BGR_YUYV), 3)
        self.writer.write(frame)

    def on_disabled(self):
        self.writer.release()
        logger.info("Recorder saved %s of video to to %s", datetime.now() - self.then, self.path)
