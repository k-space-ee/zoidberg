#!/usr/bin/env python3

from v4l2 import *
import fcntl
import mmap
import select
import time
import numpy as np
import cv2
import os

FPS = 30
ICECAST_SERVER = "193.40.101.80"
ICECAST_PORT = 8000
ICECAST_PASSWORD = "hackme"
NAMED_PIPE = "/tmp/stream%d" % os.getpid()
STORAGE_PATH = "/tmp/stream%d.webm" % os.getpid()
OPTIONAL_FILTERS = "videoscale ! video/x-raw,width=320 !"
CMD = "gst-launch-1.0 -v filesrc location='%(NAMED_PIPE)s' ! videoparse width=640 height=3840 framerate=%(FPS)d/1 format=GST_VIDEO_FORMAT_YUY2 ! videoconvert ! %(OPTIONAL_FILTERS)s vaapivp8enc keyframe-period=7 ! webmmux streamable=true ! tee name=t ! shout2send ip=%(ICECAST_SERVER)s port=%(ICECAST_PORT)d password=%(ICECAST_PASSWORD)s mount=/stream.webm t. ! filesink  async=0 location=%(STORAGE_PATH)s > /tmp/log &" % locals()

print("Storing WebM in file:", STORAGE_PATH)
print("Creating named pipe", NAMED_PIPE, "for YUYV stream")
os.mkfifo(NAMED_PIPE)

print("Executing:", CMD)
import subprocess
subprocess.call(CMD, shell=True)

print("Open stream from: http://%(ICECAST_SERVER)s:%(ICECAST_PORT)d/stream.webm" % locals())

fh = open(NAMED_PIPE, "wb")


# copy-pasta from purepy_video_capture.py follows...
webcam = '/dev/video0'
vd = open(webcam, 'rb+', buffering=0)

parm = v4l2_streamparm()
parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
parm.parm.capture.capability = V4L2_CAP_TIMEPERFRAME
fcntl.ioctl(vd, VIDIOC_G_PARM, parm)
parm.parm.capture.timeperframe.numerator = 1
parm.parm.capture.timeperframe.denominator = FPS
fcntl.ioctl(vd, VIDIOC_S_PARM, parm)

req = v4l2_requestbuffers()
req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
req.memory = V4L2_MEMORY_MMAP
req.count = 2
fcntl.ioctl(vd, VIDIOC_REQBUFS, req)

buffers = []
for ind in range(req.count):
    buf = v4l2_buffer()
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
    buf.memory = V4L2_MEMORY_MMAP
    buf.index = ind
    fcntl.ioctl(vd, VIDIOC_QUERYBUF, buf)
    mm = mmap.mmap(vd.fileno(), buf.length, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE, offset=buf.m.offset)
    buffers.append(mm)
    fcntl.ioctl(vd, VIDIOC_QBUF, buf)

buf_type = v4l2_buf_type(V4L2_BUF_TYPE_VIDEO_CAPTURE)
fcntl.ioctl(vd, VIDIOC_STREAMON, buf_type)

t0 = time.time()
max_t = 1
ready_to_read, ready_to_write, in_error = ([], [], [])
while len(ready_to_read) == 0 and time.time() - t0 < max_t:
    ready_to_read, ready_to_write, in_error = select.select([vd], [], [], max_t)

frames = 0
fps = 0
timestamp = time.time()
while True:
    frames += 1
    timestamp_begin = time.time()
    if frames > 10:
        time.time()
        fps = frames / (timestamp_begin - timestamp)
        print("fps: %.01f" % fps, end="\r")
        frames = 0
        timestamp = timestamp_begin
    buf = v4l2_buffer()
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
    buf.memory = V4L2_MEMORY_MMAP
    fcntl.ioctl(vd, VIDIOC_DQBUF, buf)
    mm = buffers[buf.index]
    stacked = np.vstack([mm, mm, mm, mm, mm, mm, mm, mm])
    combo = stacked.reshape(((8*480, 640, 2)))
    cv2.putText(combo,"fps: %.01f" % (fps), (10,30), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,128),1)
    fh.write(stacked)
    fcntl.ioctl(vd, VIDIOC_QBUF, buf)
fcntl.ioctl(vd, VIDIOC_STREAMOFF, buf_type)
vd.close()

