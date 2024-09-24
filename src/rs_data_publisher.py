#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import pyrealsense2 as rs
import numpy as np
import cv2, time, os
import D456 as camera

if __name__ == '__main__':
    imu_topic_name = "imu_stream"
    imu_topic_rate = 30
    imu_topic = (imu_topic_name, imu_topic_rate)

    depth_topic_name = "depth_stream"
    depth_topic_rate = 30
    depth_topic = (depth_topic_name, depth_topic_rate)

    # Add logic to find all connected real sense cameras, apply ID #'s, and load them
    cameras = ['d456_001'] #temporary


def init_topics(camera_id, )

