# encoding: utf-8

# Download sample video files from http://upload.itcollege.ee/robotex-test-day/

import cv2
import os
import numpy as np
import sys
from time import sleep

try:
    _, filename = sys.argv
except ValueError:
    print("Supply filename as argument!")
    sys.exit(255)

print "Opening file:", filename
cap = cv2.VideoCapture(filename)
fps = cap.get(5)
success, frame = cap.read()

# Uncomment these and guess what are they useful for
#print frame
#print frame.shape
#print frame.dtype

frameno = 0
while True:

    # Read and uncompress frame from video file
    success, frame = cap.read()
    frameno += 1
    if not success:
        break

    # Colorspace conversion
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    yuv = cv2.cvtColor(frame, cv2.COLOR_RGB2YCR_CB)

    # Find pixels in range
    mask = cv2.inRange(frame, (100,0,0), (255,100,100))
    #mask = cv2.inRange(hsv, (200,200,150), (255,255,255))

    # Remove single pixels
    mask = cv2.erode(mask, np.ones((2, 2)))

    # Cut mask out of the frame to visualize matched pixels
    cutout = cv2.bitwise_and(frame, frame, mask=mask)

    # Display framerate
    cv2.putText(frame, "frame: %d" % frameno, (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 3)

    # Stack frames
    stacked = np.vstack([
        frame,
        np.stack([mask]*3, axis=2), # See on maski näitamiseks
        cutout,
    ])

    # Find contours in the mask
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Visualize contours whose bounding rectangle is bigger than 5x5 pixels
    for cnt in contours:
        x,y,w,h = cv2.boundingRect(cnt)
        if w > 5 and h > 5:
            cv2.rectangle(stacked,(x,y),(x+w,y+h),(0,255,0),2)
            cv2.putText(stacked, "pall", (x,y), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 3)

    # Scale frame so they'd fit on screen
    zoomed = cv2.resize(stacked, None, fx=0.5,fy=0.5)

    # Display frame on the screen
    cv2.imshow("fov", zoomed)

    # Stop program if any button is pressed
    keycode = cv2.waitKey(int(1000/fps))
    if keycode >= 0:
        break

cap.release()

