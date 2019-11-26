"""
As OpenCV doesn't handle camera unplugging very well
here is an example using inotify and udev symlinks
"""

from .v4l2 import *
import fcntl
import mmap
import select
import cv2
from collections import deque
import os
from threading import Thread, Event
from time import sleep, time
import numpy as np
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
import logging
from queue import Queue, Empty
import ctypes

libc = ctypes.cdll.LoadLibrary('libc.so.6')

logger = logging.getLogger("grabber")

"""
cmd = "/sbin/udevadm", "info", "-n", self.path, "-q", "path"
sysfs_path = subprocess.check_output(cmd)[1:]
self.sysfs_path = os.path.realpath(os.path.join("/sys", sysfs_path, "..", ".."))
print("Unpowering:", self.sysfs_path)
with open(os.path.join(self.sysfs_path, "authorized"), "w") as fh:
    fh.write("0")
"""

# TODO: why 4 channels
BLANK = np.zeros((480, 320, 4), dtype=np.uint8)

observer = Observer()
observer.start()


class CaptureHotplugHandler(FileSystemEventHandler):
    def __init__(self, grabber):
        self.grabber = grabber
        FileSystemEventHandler.__init__(self)

    def on_created(self, event):
        if event.src_path != self.grabber.path:
            return
        logger.info("Attached: %s", event.src_path)
        self.grabber.wake.set()


class Grabber(Thread):
    def __init__(self, device, fps=30, exposure=None, gain=None, saturation=None, name=None):
        Thread.__init__(self)
        logger.info("Starting grabber for: %s", device)
        self.path = os.path.join("/dev/v4l/by-path", device)

        self.fps = fps
        self.exposure = exposure
        self.gain = gain
        self.saturation = saturation

        self.ready = Event()  # Used to tell consumers that new frame is available
        self.ready.clear()

        self.wake = Event()  # Used by observer to tell grabber that capture device is available
        self.wake.clear()

        self.name = name or device

        self.daemon = True
        self.running = True  # Whether thread is running
        self.alive = False  # Whether frames are being captured
        self.vd = 0  # Video capture descriptor
        self.latencies = deque(maxlen=10)
        self.timestamp = time()
        self.frame_count = 0
        self.error_count = 0 if os.path.exists(self.path) else 1
        self.dropped_count = 0

        self.queues = set()
        self.frame = None
        if observer:
            try:
                observer.schedule(CaptureHotplugHandler(self), "/dev/v4l/by-path", recursive=False)
            except:
                logger.error("Hotplug disabled for %s", self.path)
        else:
            logger.error("Hotplug disabled for %s", self.path)

    def get_queue(self):
        """
        Create queue for new consumer
        """
        q = Queue(maxsize=1)
        self.queues.add(q)
        return q

    def open(self):
        logger.info("Opening %s requesting %d fps", self.path, self.fps)
        vd = open(os.path.realpath(self.path), 'rb+', buffering=0)

        # Query camera capabilities
        cp = v4l2_capability()
        fcntl.ioctl(vd, VIDIOC_QUERYCAP, cp)
        self.driver = "".join((chr(c) for c in cp.driver if c))

        # logger.info("Disabling auto white balance for %s", self.path)
        ctrl = v4l2_control()
        ctrl.id = V4L2_CID_AUTO_WHITE_BALANCE
        ctrl.value = 0
        fcntl.ioctl(vd, VIDIOC_S_CTRL, ctrl)

        if self.saturation is not None:
            # logger.info("Setting saturation for %s to %d", self.path, self.saturation)
            ctrl = v4l2_control()
            ctrl.id = V4L2_CID_SATURATION
            ctrl.value = self.saturation
            fcntl.ioctl(vd, VIDIOC_S_CTRL, ctrl)

        if self.exposure is not None:
            # logger.info("Setting exposure for %s to %d", self.path, self.exposure)
            # Disable auto exposure
            ctrl = v4l2_control()
            ctrl.id = V4L2_CID_EXPOSURE_AUTO
            ctrl.value = V4L2_EXPOSURE_MANUAL
            fcntl.ioctl(vd, VIDIOC_S_CTRL, ctrl)

            # Set exposure manually
            ctrl = v4l2_control()
            ctrl.id = V4L2_CID_EXPOSURE
            ctrl.value = self.exposure
            fcntl.ioctl(vd, VIDIOC_S_CTRL, ctrl)

        else:
            # Enable auto exposure
            # logger.info("Setting auto exposure for %s", self.path)
            ctrl = v4l2_control()
            ctrl.id = V4L2_CID_EXPOSURE_AUTO
            ctrl.value = V4L2_EXPOSURE_AUTO
            fcntl.ioctl(vd, VIDIOC_S_CTRL, ctrl)

        # Flip camera horizontally
        ctrl = v4l2_control()
        ctrl.id = V4L2_CID_HFLIP
        ctrl.value = 0
        fcntl.ioctl(vd, VIDIOC_S_CTRL, ctrl)

        # Flip camera vertically
        ctrl = v4l2_control()
        ctrl.id = V4L2_CID_VFLIP
        ctrl.value = 1
        fcntl.ioctl(vd, VIDIOC_S_CTRL, ctrl)

        if self.gain is not None:
            # Disable autogain
            # logger.info("Setting gain for %s to %d", self.path, self.gain)
            ctrl = v4l2_control()
            ctrl.id = V4L2_CID_AUTOGAIN
            ctrl.value = 0
            fcntl.ioctl(vd, VIDIOC_S_CTRL, ctrl)

            # Set gain manually
            ctrl = v4l2_control()
            ctrl.id = V4L2_CID_GAIN
            ctrl.value = self.gain
            fcntl.ioctl(vd, VIDIOC_S_CTRL, ctrl)

        else:
            # Enable autogain
            # logger.info("Setting autogain for %s", self.path)
            ctrl = v4l2_control()
            ctrl.id = V4L2_CID_AUTOGAIN
            ctrl.value = 1
            fcntl.ioctl(vd, VIDIOC_S_CTRL, ctrl)

        if self.fps is not None:
            # Set framerate
            parm = v4l2_streamparm()
            parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
            parm.parm.capture.capability = V4L2_CAP_TIMEPERFRAME
            fcntl.ioctl(vd, VIDIOC_G_PARM, parm)  # get current camera settings
            parm.parm.capture.timeperframe.numerator = 1
            parm.parm.capture.timeperframe.denominator = self.fps
            fcntl.ioctl(vd, VIDIOC_S_PARM, parm)  # change camera capture settings

        # Initalize mmap with multiple buffers
        req = v4l2_requestbuffers()
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
        req.memory = V4L2_MEMORY_MMAP
        req.count = 4  # nr of buffer frames
        try:
            fcntl.ioctl(vd, VIDIOC_REQBUFS, req)
        except Exception as e:
            vd.close()
            raise e

        self.buffers = []

        # Setup buffers
        for i in range(req.count):
            buf = v4l2_buffer()
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
            buf.memory = V4L2_MEMORY_MMAP
            buf.index = i
            fcntl.ioctl(vd, VIDIOC_QUERYBUF, buf)
            mm = mmap.mmap(vd.fileno(), buf.length, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE,
                           offset=buf.m.offset)
            self.buffers.append(mm)
            fcntl.ioctl(vd, VIDIOC_QBUF, buf)

        # Start streaming
        fcntl.ioctl(vd, VIDIOC_STREAMON, v4l2_buf_type(V4L2_BUF_TYPE_VIDEO_CAPTURE))

        # Wait cameras to get ready
        t0 = time()
        max_t = 1
        ready_to_read, ready_to_write, in_error = ([], [], [])
        while len(ready_to_read) == 0 and time() - t0 < max_t:
            ready_to_read, ready_to_write, in_error = select.select([vd], [], [], max_t)
        return vd

    def run(self):
        wait_count = 0
        while self.running:
            sleep(0.005)
            self.ready.clear()

            if not self.vd:
                # give some time for cameras to recover
                sleep(0.1)
                # Check if /dev/v4l/by-path/bla symlink exists
                if not os.path.exists(self.path):
                    # logger.info("Waiting for %s to become available", self.path)
                    self.wake.wait(timeout=0.2)
                    self.wake.clear()
                    wait_count += 1
                    if wait_count < 2:
                        continue

                else:
                    try:
                        self.vd = self.open()
                    except OSError as e:
                        logger.error("OSError: Failed to open: %s\n%s", self.path, e)
                        self.vd = None
                    except Exception as e:
                        logger.error("Exception: Failed to open: %s\n%s", self.path, e)
                        self.vd = None

                if not self.vd:
                    self.die("FUCK this")
                    os.system('rosnode kill octocamera')

            wait_count = 0
            # get image from the driver queue
            buf = v4l2_buffer()
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
            buf.memory = V4L2_MEMORY_MMAP

            try:
                fcntl.ioctl(self.vd, VIDIOC_DQBUF, buf)  # deque from v4l
                mm = self.buffers[buf.index]

                self.frame = np.asarray(mm, dtype=np.uint8).reshape((480, 320, 4))

                for output_queue in self.queues:
                    try:
                        output_queue.get_nowait()
                    except Empty:
                        pass
                    finally:
                        output_queue.put((self.frame,))

                self.ready.set()
                now = time()
                delta = now - self.timestamp
                if self.fps is not None:
                    if delta > 2.0 / self.fps:
                        self.dropped_count += 1
                self.latencies.append(delta)
                self.timestamp = now
                self.alive = True
                self.frame_count += 1
                sleep(0.005)
                fcntl.ioctl(self.vd, VIDIOC_QBUF, buf)  # requeue the buffer
            except OSError:
                self.die("Camera unplugged")
                os.system('rosnode kill octocamera')
                self.frame = None
            except IOError:
                self.die("Camera unplugged")
                os.system('rosnode kill octocamera')
                logger.info("%s dropped frame!", self.path)
                self.dropped_count += 1
            except BaseException:
                self.die("BaseException")
                os.system('rosnode kill octocamera')

        # Graceful shutdown
        if self.vd:
            fcntl.ioctl(self.vd, VIDIOC_STREAMOFF, v4l2_buf_type(V4L2_BUF_TYPE_VIDEO_CAPTURE))
        self.die("Graceful shutdown")

    def die(self, reason):
        if self.vd:
            fcntl.ioctl(self.vd, VIDIOC_STREAMOFF, v4l2_buf_type(V4L2_BUF_TYPE_VIDEO_CAPTURE))
            try:
                self.vd.close()
            except:
                pass
            
        self.vd = None
        self.alive = False
        self.error_count += 1
        self.frame = None
        self.ready.clear()
        self.running = False
        # clear cached frames
        logger.info("%s dying because %s", self.path, reason)
        sleep(0.1)

    def restart(self):
        self.die("restart")

    def stop(self):
        self.running = False
        self.wake.set()

    def disable(self):
        if self.vd:
            self.vd.close()
        self.vd = None
        self.frame = None


class PanoramaGrabber(Thread):
    def __init__(self, config):
        Thread.__init__(self)
        self.last_product = 0

        global_config = config.prop("global")
        kwargs = dict((k, v) for k, v in global_config.items() if k in ("fps", "gain", "exposure", "saturation"))
        self.slaves = []
        for i in range(1, global_config.get("cameras", 0) + 1):
            self.slaves.append(Grabber(config.get("camera%d" % i, {}).get("path"), name="camera%d" % i, **kwargs))

        self.tid = 0
        self.running = False
        self.alive = True

        self.latency = deque(maxlen=10)
        self.rate = deque(maxlen=10)

        self.rotate = config.get("global", {}).get("rotate")
        self.queues = set()
        self.input_queues = [slave.get_queue() for slave in self.slaves]  # [0].get_queue()

    def start(self):
        for slave in self.slaves:
            slave.start()
        Thread.start(self)

    def get_queue(self, lossy=True):
        """
        Create queue for new consumer
        """
        q = Queue(maxsize=1)
        self.queues.add(q)
        return q

    def get_panorama(self):
        """
        Return view of captured frames merged as one
        """
        slaves = self.slaves + self.slaves[:1]
        frames = [slave.frame if slave.alive else BLANK for slave in slaves]

        if self.rotate:
            return np.rot90(np.vstack(frames), 3).copy()
        else:
            return np.hstack(frames).copy()

    def get_panorama_hsv(self):
        """
        Return view of captured frames merged as one
        """
        slaves = self.slaves + self.slaves[:1]
        frames = [slave.hsv if slave.alive else BLANK for slave in slaves]

        if self.rotate:
            return np.rot90(np.vstack(frames), 3).copy()
        else:
            return np.hstack(frames).copy()

    def get_tiled(self):
        """
        Return view of captured frames merged as one
        """
        half = len(self.slaves) >> 1
        frames = [slave.frame if slave.alive else BLANK for slave in self.slaves]
        if self.rotate:
            return np.vstack([
                np.swapaxes(np.vstack(frames[:half]), 1, 0),
                np.swapaxes(np.vstack(frames[half:]), 1, 0)
            ])
        else:
            return np.vstack([
                np.hstack(frames[:half]),
                np.hstack(frames[half:])
            ])

    def run(self):
        self.tid = libc.syscall(186)
        logger.info("%s thread spawned with PID %d", self.__class__.__name__, self.tid)
        self.running = True

        while self.running:
            then = time()
            # Synchronize producers
            products = [(queue.get() if slave.alive else (BLANK,)) for queue, slave in
                        zip(self.input_queues, self.slaves)]
            products.append(products[0])
            then2 = time()
            now = time()
            self.latency.append(now - then2)
            self.rate.append(now - then)

            stacked = np.vstack([frame for frame, in products])
            # Pump panorama frames to consumers
            for queue in self.queues:
                try:
                    queue.get_nowait()
                except Empty:
                    pass
                finally:
                    queue.put((stacked,))
                self.last_product = time()

    def stop(self):
        self.running = False
        for slave in self.slaves:
            slave.stop()
        for slave in self.slaves:
            slave.join()


if __name__ == "__main__":
    from recorder import Recorder

    grabber = PanoramaGrabber()
    grabber.start()
    recorder = Recorder(grabber)
    recorder.enable()

    queue = grabber.get_queue()
    try:
        while True:
            yuyv, = queue.get()
            frame = np.swapaxes(cv2.cvtColor(yuyv.reshape((-1, 640, 2)), cv2.COLOR_YUV2BGR_YUYV), 0, 1)
            frame = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)

            height, width, depth = frame.shape
            step = width / len(grabber.slaves)

            mask = cv2.inRange(yuyv, (64, 0, 64, 160), (255, 128, 255, 255))
            cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
            balls = set()
            for c in cnts:
                y, x, h, w = cv2.boundingRect(c)

                if w < 2 or h < 2:
                    continue

                # Adjust for the fact that we have 320 YUYV pixels
                x = x >> 1
                w = w >> 1
                cx = x + (w >> 1)
                cy = y + (h >> 1)
                radius = w >> 1 if w > h else h >> 1

                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 255, 255), 3)
                cv2.circle(frame, (cx, cy), radius, (255, 255, 255), 3)

            for index, slave in enumerate(grabber.slaves):
                x = int(index * step + 20)
                _, _, _, slug, _ = os.path.basename(slave.path).split("-", 4)
                cv2.putText(frame, slug, (x, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                if slave.alive:
                    cv2.putText(frame, "fps: %.01f" % (len(slave.latencies) / sum(slave.latencies)), (x, 80),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1)
                else:
                    cv2.putText(frame, "camera dead", (x, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1)
                cv2.putText(frame, "errors: %d frames: %d" % (slave.error_count, slave.frame_count), (x, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            cv2.imshow('img', frame)
            if cv2.waitKey(1) >= 0:
                break

    except KeyboardInterrupt:
        pass
    recorder.stop()
    grabber.stop()
    observer.stop()
    observer.join()
    recorder.join()
