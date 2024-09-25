#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np
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
    if topic_name == 'imu_stream':
        topic = rospy.Publisher(f"/camera_{camera.get_id()}/{topic_name}", String, queue_size=10)  # Publisher for the camera imu stream
    else:
        topic = rospy.Publisher(f"/camera_{camera.get_id()}/{topic_name}", Image, queue_size=10)  # Publisher for the camera image streams
    
    rate = rospy.Rate(pub_rate)  # Set the rate of publishing

    while not rospy.is_shutdown():
        # Get stream data and publish to the appropriate topic
        data = stream_data_getter()  # Retrieve data from the camera (e.g., pose, RGB image, etc.)
        topic.publish(data)  # Publish the data to the ROS topic
        rate.sleep()  # Maintain the specified publishing rate

## @brief Main function to initialize cameras and launch ROS nodes for each enabled stream.
##
## This function finds all connected Intel RealSense cameras and creates threads for publishing each enabled stream (IMU, RGB, Depth, IR) 
## on corresponding ROS topics. The function uses the D456 class to access the cameras.
if __name__ == '__main__':
    
    bridge = CvBridge()

    # TODO: Add logic to discover and initialize multiple RealSense cameras
    cameras = [rs_camera.D456("d456_001")]  # Temporary list containing one camera

    # Initialize an empty list to store threads for each camera stream node
    threads = []

    # Loop through each camera and setup the ROS topics for the enabled streams
    for camera in cameras:
                
        if 'imu_stream' in camera.get_enabled_streams():
            rate = min(int(camera.get_accel_rate()), int(camera.get_gyro_rate()))

            # Create and start a new thread for publishing IMU data
            threads.append(threading.Thread(target=publish_stream, args=(camera, 'imu_stream', rate , camera.get_pose())))

        if 'depth_stream' in camera.get_enabled_streams() and 'rgb_stream' in camera.get_enabled_streams():
            images = camera.get_rgbd_image_pair()
            rate = camera.get_frame_rate()

            color_ros_image = bridge.cv2_to_imgmsg(images[0], encoding="bgr8")
            depth_ros_image = bridge.cv2_to_imgmsg(images[1], encoding="16UC1")

            threads.append(threading.Thread(target=publish_stream, args=(camera, 'color_stream', rate, color_ros_image)))
            threads.append(threading.Thread(target=publish_stream, args=(camera, 'depth_stream', rate, depth_ros_image)))

    for thread in threads:
        thread.start()

    rospy.spin()
