import cv2
import numpy as np
import pyrealsense2 as rs
import time, os

# TODO
# 1. find all real sense cameras
# 2. Index them by serial number

captures = []

# captures.append(cv2.VideoCapture(3))
# captures.append(cv2.VideoCapture(4))

cap1 = cv2.VideoCapture(0)
cap2 = cv2.VideoCapture(1)

while True:
    ret1, frame1 = cap1.read()
    ret2, frame2 = cap2.read()
    
    if ret1:
        cv2.imshow('Camera 1', frame1)

    if ret2:
        cv2.imshow('Camera 2', frame2)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap1.release()
cap2.release()
cv2.destroyAllWindows()
