#!/usr/bin/env python3

from v4l2 import *
import logging
import fcntl
import mmap
import select
import time
import cv2
import numpy

logger = logging.getLogger("grabber")

class Grabber(object):
    def __init__(self, device, fps=30, exposure=None, gain=None, saturation=None, name=None, vflip=False, hflip=False):
        logger.info("Starting grabber for:", device)
        self.path = device if device.startswith("/") else os.path.join("/dev/v4l/by-path", device)
        self.fps = fps
        self.exposure = exposure
        self.gain = gain
        self.saturation = saturation
        self.vflip = vflip
        self.hflip = hflip
        self.handle = None

    def open(self):
        self.handle = open(self.path, 'rb+', buffering=0)

        cp = v4l2_capability()
        fcntl.ioctl(self.handle, VIDIOC_QUERYCAP, cp)

        fmt = v4l2_format()
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
        fcntl.ioctl(self.handle, VIDIOC_G_FMT, fmt)  # get current settings
        print("width:", fmt.fmt.pix.width, "height", fmt.fmt.pix.height)
        print("pxfmt:", "V4L2_PIX_FMT_YUYV" if fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUYV else fmt.fmt.pix.pixelformat)
        print("bytesperline:", fmt.fmt.pix.bytesperline)
        print("sizeimage:", fmt.fmt.pix.sizeimage)
        fcntl.ioctl(self.handle, VIDIOC_S_FMT, fmt)  # set whatever default settings we got before

        parm = v4l2_streamparm()
        parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
        parm.parm.capture.capability = V4L2_CAP_TIMEPERFRAME
        fcntl.ioctl(self.handle, VIDIOC_G_PARM, parm) # get current camera settings
        # set framerate to 60fps or 1/60
        parm.parm.capture.timeperframe.numerator = 1
        parm.parm.capture.timeperframe.denominator = 5
        print("parm.capture.timeperframe: 1/60 fps")
        fcntl.ioctl(self.handle, VIDIOC_S_PARM, parm)  # change camera capture settings

        logger.info("Disabling auto white balance for %s", self.path)
        ctrl = v4l2_control()
        ctrl.id = V4L2_CID_AUTO_WHITE_BALANCE
        ctrl.value = 0
        fcntl.ioctl(self.handle, VIDIOC_S_CTRL, ctrl)


        if self.saturation is not None:
            logger.info("Setting saturation for %s to %d", self.path, self.saturation)
            ctrl = v4l2_control()
            ctrl.id = V4L2_CID_SATURATION
            ctrl.value = self.saturation
            fcntl.ioctl(self.handle, VIDIOC_S_CTRL, ctrl)

        if self.exposure is not None:
            logger.info("Setting exposure for %s to %d", self.path, self.exposure)
            # Disable auto exposure
            ctrl = v4l2_control()
            ctrl.id = V4L2_CID_EXPOSURE_AUTO
            ctrl.value = V4L2_EXPOSURE_MANUAL
            fcntl.ioctl(self.handle, VIDIOC_S_CTRL, ctrl)

            # Set exposure manually
            ctrl = v4l2_control()
            ctrl.id = V4L2_CID_EXPOSURE
            ctrl.value = self.exposure
            fcntl.ioctl(self.handle, VIDIOC_S_CTRL, ctrl)

        else:
            # Enable auto exposure
            logger.info("Setting auto exposure for %s", self.path)
            ctrl = v4l2_control()
            ctrl.id = V4L2_CID_EXPOSURE_AUTO
            ctrl.value = V4L2_EXPOSURE_AUTO
            fcntl.ioctl(self.handle, VIDIOC_S_CTRL, ctrl)

        # Flip camera horizontally
        ctrl = v4l2_control()
        ctrl.id = V4L2_CID_HFLIP
        ctrl.value = self.hflip
        fcntl.ioctl(self.handle, VIDIOC_S_CTRL, ctrl)

        # Flip camera vertically
        ctrl = v4l2_control()
        ctrl.id = V4L2_CID_VFLIP
        ctrl.value = self.vflip
        fcntl.ioctl(self.handle, VIDIOC_S_CTRL, ctrl)

        if self.gain is not None:
            # Disable autogain
            logger.info("Setting gain for %s to %d", self.path, self.gain)
            ctrl = v4l2_control()
            ctrl.id = V4L2_CID_AUTOGAIN
            ctrl.value = 0
            fcntl.ioctl(self.handle, VIDIOC_S_CTRL, ctrl)

            # Set gain manually
            ctrl = v4l2_control()
            ctrl.id = V4L2_CID_GAIN
            ctrl.value = self.gain
            fcntl.ioctl(self.handle, VIDIOC_S_CTRL, ctrl)

        else:
            # Enable autogain
            logger.info("Setting autogain for %s", self.path)
            ctrl = v4l2_control()
            ctrl.id = V4L2_CID_AUTOGAIN
            ctrl.value = 1
            fcntl.ioctl(self.handle, VIDIOC_S_CTRL, ctrl)


        if self.fps is not None:
            # Set framerate
            parm = v4l2_streamparm()
            parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
            parm.parm.capture.capability = V4L2_CAP_TIMEPERFRAME
            fcntl.ioctl(self.handle, VIDIOC_G_PARM, parm) # get current camera settings
            parm.parm.capture.timeperframe.numerator = 1
            parm.parm.capture.timeperframe.denominator = self.fps
            fcntl.ioctl(self.handle, VIDIOC_S_PARM, parm) # change camera capture settings

        req = v4l2_requestbuffers()
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
        req.memory = V4L2_MEMORY_MMAP
        req.count = 2  # nr of buffer frames
        fcntl.ioctl(self.handle, VIDIOC_REQBUFS, req)  # tell the driver that we want some buffers

        self.buffers = []

        for ind in range(req.count):
            buf = v4l2_buffer()
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
            buf.memory = V4L2_MEMORY_MMAP
            buf.index = ind
            fcntl.ioctl(self.handle, VIDIOC_QUERYBUF, buf)
            mm = mmap.mmap(self.handle.fileno(), buf.length, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE, offset=buf.m.offset)
            self.buffers.append(mm)
            fcntl.ioctl(self.handle, VIDIOC_QBUF, buf)

        buf_type = v4l2_buf_type(V4L2_BUF_TYPE_VIDEO_CAPTURE)
        fcntl.ioctl(self.handle, VIDIOC_STREAMON, buf_type)

        t0 = time.time()
        max_t = 1
        ready_to_read, ready_to_write, in_error = ([], [], [])
        while len(ready_to_read) == 0 and time.time() - t0 < max_t:
            ready_to_read, ready_to_write, in_error = select.select([self.handle], [], [], max_t)

    def pop(self):
        buf = v4l2_buffer()
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
        buf.memory = V4L2_MEMORY_MMAP
        fcntl.ioctl(self.handle, VIDIOC_DQBUF, buf)
        mm = self.buffers[buf.index]

        uv = numpy.asarray(mm, numpy.uint8)[1::2].reshape(((480, 320, 2)))
        uv = numpy.repeat(uv, 2, axis=1)  # kills perf but fixes aspect ratio
        #blurred_uv = cv2.blur(uv, (4,4))  # kills perf but smooths the picture
        blurred_uv = uv
        mask = cv2.inRange(blurred_uv, (60, 160), (90, 255))  ## FILTER THE COLORS!!
        #mask = cv2.dilate(mask, None, iterations=2) # kills perf, removes sparkling


        frame = numpy.asarray(mm, numpy.uint8).reshape((480, 640, 2))
        frame = cv2.cvtColor(frame, cv2.COLOR_YUV2BGR_YUYV)

        fcntl.ioctl(self.handle, VIDIOC_QBUF, buf)  # requeue the buffer
        return frame

    def close(self):
        fcntl.ioctl(self.handle, VIDIOC_STREAMOFF, buf_type)
        self.handle.close()
        self.handle = None


class QuadGrabber(object):
    def __init__(self, a="/dev/video0", b="/dev/video1", c="/dev/video2", d="/dev/video3"):
        self.grabbers = Grabber(c), Grabber(b), Grabber(d), Grabber(a)

    def open(self):
        for grabber in self.grabbers:
            grabber.open()

    def pop(self):
        return numpy.vstack([
            numpy.hstack([self.grabbers[0].pop(), self.grabbers[1].pop()]),
            numpy.hstack([self.grabbers[2].pop(), self.grabbers[3].pop()])
        ])

    def close(self):
        for grabber in self.grabbers:
            grabber.close()


from flask import Flask, Response
app = Flask(__name__)

@app.route('/')
def hello_world():

    grabber = QuadGrabber()
    grabber.open()
    frame = grabber.pop()
    def generator():
        while True:
            ret, jpeg = cv2.imencode('.jpg', frame, (cv2.IMWRITE_JPEG_QUALITY, 50))
            buf = jpeg.tostring()
            yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
            yield buf
            yield b'\r\n\r\n'
    return Response(generator(), mimetype='multipart/x-mixed-replace; boundary=frame')

app.run(debug=True, host="0.0.0.0")
