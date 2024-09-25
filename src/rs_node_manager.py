#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import CvBridge
import pyrealsense2 as rs
import numpy as np
import cv2, time, os
import D456 as rs_camera
import threading

## @file rs_stream_publisher.py
## @brief ROS node for publishing Intel RealSense camera streams.
##
## This script is responsible for capturing stream data from Intel RealSense cameras (e.g., D456)
## and publishing the data to different ROS topics. It supports RGB, depth, infrared, and IMU streams.
## The node allows multiple threads for parallel publishing of different streams.
##
## @author Jason Davis
## @date 9/24/2024
## @license GPL 3.0


## @brief Publishes the stream data to the given ROS topic.
##
## This function continuously retrieves stream data from the camera and publishes it to the appropriate ROS topic at a specified rate.
##
## @param camera Instance of the D456 class to retrieve data from.
## @param topic_name Name of the ROS topic to publish the data.
## @param pub_rate Rate at which the topic will be published (Hz).
## @param stream_data_getter Function that retrieves the stream data (e.g., pose, RGB, depth).
def publish_stream(camera, topic_name, pub_rate, stream_data_getter):
    rospy.init_node(f"rs_{camera.get_id()}/{topic_name}_node", anonymous=True)  # Initialize the ROS node
    topic = rospy.Publisher(f"/camera_{camera.get_id()}/{topic_name}", String, queue_size=10)  # Publisher for the camera stream
    rate = rospy.Rate(pub_rate)  # Set the rate of publishing

    while not rospy.is_shutdown():
        # Get stream data and publish to the appropriate topic
        data = stream_data_getter()  # Retrieve data from the camera (e.g., pose, RGB image, etc.)
        topic.publish(data)  # Publish the data to the ROS topic
        rate.sleep()  # Maintain the specified publishing rate

## @brief Sets up the camera's enabled streams and prepares them for publishing.
##
## This function checks the enabled streams of the camera and returns the relevant topic parameters such as topic name and publishing rate.
##
## @param camera Instance of the D456 class to retrieve stream configurations from.
## @return List of tuples containing topic names and their corresponding publishing rates.
def setup(camera):
    
    topic_params = []  # List to store topic names and their publish rates

    # Check if the IMU stream is enabled and append it to topic_params
    if 'imu_stream' in camera.get_enabled_streams():
        imu_topic_name = "imu_stream"
        imu_topic_rate = min(int(camera.get_accel_rate()), int(camera.get_gyro_rate()))  # Use the lower rate between accelerometer and gyroscope
        topic_params.append((imu_topic_name, imu_topic_rate))

    # Check if the depth stream is enabled and append it to topic_params
    if 'depth_stream' in camera.get_enabled_streams():
        depth_topic_name = "depth_stream"
        depth_topic_rate = camera.get_frame_rate()  # Depth stream rate is the same as the camera's frame rate
        topic_params.append((depth_topic_name, depth_topic_rate))

    # Check if the RGB stream is enabled and append it to topic_params
    if 'rgb_stream' in camera.get_enabled_streams():
        rgb_topic_name = "rgb_stream"
        rgb_topic_rate = camera.get_frame_rate()  # RGB stream rate is also tied to the camera's frame rate
        topic_params.append((rgb_topic_name, rgb_topic_rate))

    # Check if the IR stream is enabled and append it to topic_params
    if 'ir_stream' in camera.get_enabled_streams():
        ir_topic_name = "ir_stream"
        ir_topic_rate = camera.get_frame_rate()  # IR stream rate follows the frame rate
        topic_params.append((ir_topic_name, ir_topic_rate))

    return topic_params  # Return the list of topic names and their rates

## @brief Main function to initialize cameras and launch ROS nodes for each enabled stream.
##
## This function finds all connected Intel RealSense cameras and creates threads for publishing each enabled stream (IMU, RGB, Depth, IR) 
## on corresponding ROS topics. The function uses the D456 class to access the cameras.
if __name__ == '__main__':
    
    # TODO: Add logic to discover and initialize multiple RealSense cameras
    cameras = [rs_camera.D456("d456_001")]  # Temporary list containing one camera

    # Initialize an empty list to store threads for each camera stream node
    nodes = []

    # Loop through each camera and setup the ROS topics for the enabled streams
    for camera in cameras:
        # Use all available streams from the camera
        topic_params = setup(camera)

        # Iterate through each enabled stream of the camera
        for stream in camera.get_enabled_streams():
                 
            if stream == 'imu_stream':
                # Create and start a new thread for publishing IMU data
                threading.Thread(target=publish_stream, args=(camera, stream, topic_params[1], camera.get_pose())).start()

            # TODO: Add threads for other streams (RGB, Depth, IR) as needed
