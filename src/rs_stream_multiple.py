import cv2
import numpy as np
import pyrealsense2 as rs
import time, os, re, subprocess

# TODO
# 1. find all real sense cameras - DONE
# 2. Index them by serial number

# Constants
RESOLUTION_WIDTH = 1280
RESOLUTION_HEIGHT = 720
FRAME_RATE = 30 # Max supported frame rate is 30 FPS

def find_rs_devices(target_model=""):
    
    rs_devices = []

    ctx = rs.context()
    for device in ctx.query_devices():
        device.group_dict()
        rs_devices.append(device)

    return rs_devices

def init_rs_pipeline(rs_devices):
    
    pipelines = []

    for device in rs_devices:

        pipeline = rs.pipeline()
        config = rs.config()

        config.enable_stream(rs.stream.depth, RESOLUTION_WIDTH, RESOLUTION_HEIGHT,rs.format.z16, FRAME_RATE)
        pipeline.start(config)
        pipelines.append((device['id'], pipeline))

    return pipelines

if __name__ == '__main__':
    
    rs_devices = find_rs_devices()              # find rs cameras on the bus
    pipelines = init_rs_pipeline(rs_devices)    # start a separate pipeline for each camera

    quit = False

    while not quit:

        for pipeline in pipelines:

            frame = pipeline[1].poll_for_frames()

            depth_frame = frame.get_depth_frame()

            if not depth_frame:
                continue

            depth_image = np.asanyarray(depth_frame.get_data())
            depth_image = cv2.convertScaleAbs(depth_image, alpha=0.03)
            cv2.imshow(pipeline[0], depth_image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            quit = True
            break

    cv2.destroyAllWindows()
