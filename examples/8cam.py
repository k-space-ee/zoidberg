import cv2
import numpy as np
from time import time, sleep

captures = []

for j in range(8):
    video = cv2.VideoCapture(j)
    video.set(3, 640)
    video.set(4, 480)
    success, _ = video.read()
    if not success:
        continue
    captures.append(video)
    print("camera {} initialized {}".format(j, success))

if not len(captures) == 8:
    raise Exception("you are stupid")

count, last = 0, time()
while True:
    now = time()
    if now - last > 2:
        print("%d fps" % (count * 1.0 / (now - last)))
        last = now
        count = 0
    count += 1

    frames = [cap.read()[1] for cap in captures]
    frames = [cv2.resize(frame, (0, 0), fx=0.7, fy=0.7) for frame in frames]
    
    stackedA = np.hstack(frames[:4])
    stackedB = np.hstack(frames[4:])
    stacked = np.vstack([stackedA, stackedB])

    cv2.imshow('just a name', stacked)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
