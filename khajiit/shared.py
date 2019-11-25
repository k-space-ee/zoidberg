from threading import Thread
from time import time, sleep

import SharedArray as sa
import numpy as np


def get_image_publisher(channel: str, shape: tuple, dtype) -> np.ndarray:
    # Create an array in shared memory.
    short_name = channel.split("://")[-1]
    mapping = {e.name.decode(): e for e in sa.list()}
    if short_name in mapping:
        array = mapping[short_name]
        if array.dtype == dtype and array.dims == shape:
            return sa.attach(channel)
        sa.delete(short_name)

    return sa.create(channel, shape, dtype)


def attach(channel: str) -> np.ndarray:
    frame: np.ndarray = sa.attach(channel)
    return frame


class ImageSubscriber(Thread):
    """
    Spin-wait, could be replaced with pub-sub events
    """

    def __init__(self, channel: str, callback: callable = None, sync_gain: float = 0.90, **kwargs) -> None:
        Thread.__init__(self, **kwargs)
        self.callback = callback
        self.sync_gain = sync_gain
        self.max_delay = 0.030
        self.channel = channel
        self.channel_reset_delay = 60

    def run(self) -> None:
        frame: np.ndarray = sa.attach(self.channel)
        old = frame.copy()

        start = time()
        counter = 1

        while True:
            counter += 1

            if counter % self.channel_reset_delay == 0:
                frame: np.ndarray = sa.attach(self.channel)

            if not np.array_equal(old, frame):
                old = frame.copy()

                estimated_cap_delay = time() - start
                self.callback and self.callback(old, estimated_cap_delay)
                start = time()
                sleep(min(self.max_delay, estimated_cap_delay * self.sync_gain))
            else:
                sleep(0.001)


if __name__ == '__main__':
    import cv2 as cv

    SHOW = True


    def receive(frame, delay):
        print(1 / delay)

        if SHOW:
            cv.imshow('Client', frame)
            if cv.waitKey(1) == 27:
                exit()


    sub = ImageSubscriber("shm://depth-color", receive)
    sub.start()
