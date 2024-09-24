import pyrealsense2 as rs
import numpy as np
import cv2, time, os
from datetime import datetime
import argparse

def parse_arguments():
    parser = argparse.ArgumentParser(description='Capture RGB and Depth images from RealSense camera.')
    parser.add_argument('--runtime', type=int, default=0.5, help='Specify the script runtime in minutes. Default is 30 seconds.')
    return parser.parse_args()

if __name__ == '__main__':
    
    # Parse command line arguments
    args = parse_arguments()
    
    # Convert runtime from minutes to seconds
    RECORD_DURATION = args.runtime * 60
    
    # Constants
    RESOLUTION_WIDTH = 1280
    RESOLUTION_HEIGHT = 720
    FRAME_RATE = 30 # This is the camera frame rate, not the image capture rate
    DISPLAY_FRAMES = False
    IMAGE_CAPTURE_INTERVAL = 5 # seconds

    # Create /media/ directory if it doesn't exist
    media_dir = os.path.join(os.path.expanduser("~"), "real_sense_media")
    if not os.path.exists(media_dir):
        os.makedirs(media_dir)
    
    # Create /media/<DATE>/ subdirectory with the format YYYY_MM_DD
    date_str = datetime.now().strftime("%Y_%m_%d")
    date_dir = os.path.join(media_dir, date_str)
    if not os.path.exists(date_dir):
        os.makedirs(date_dir)

    # Initialize RealSense Pipeline
    pipeline = rs.pipeline()
    config = rs.config()

    # Enable color and depth streams
    config.enable_stream(rs.stream.color, RESOLUTION_WIDTH, RESOLUTION_HEIGHT, rs.format.bgr8, FRAME_RATE)
    config.enable_stream(rs.stream.depth, RESOLUTION_WIDTH, RESOLUTION_HEIGHT, rs.format.z16, FRAME_RATE)

    # Start the pipeline
    pipeline.start(config)

    # Get the depth scale to convert depth data to meters
    depth_sensor = pipeline.get_active_profile().get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()

    # Start recording
    start_time = time.time()
    last_capture_time = start_time  # Track the last time an image was captured

    try:
        while time.time() - start_time < RECORD_DURATION:
            # Wait for a new set of frames
            frames = pipeline.wait_for_frames()

            # Get the RGB frame
            color_frame = frames.get_color_frame()

            # Get the depth frame
            depth_frame = frames.get_depth_frame()

            if not color_frame or not depth_frame:
                continue

            # Convert images to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            # Convert depth image to 8-bit for saving
            depth_image_8bit = cv2.convertScaleAbs(depth_image, alpha=0.03)

            # Get the current time
            current_time = time.time()

            # Capture image every 5 seconds
            if current_time - last_capture_time >= IMAGE_CAPTURE_INTERVAL:
                # Save the RGB and Depth images with UNIX time as the suffix
                timestamp = str(int(current_time))

                rgb_image_path = os.path.join(date_dir, f'rgb_{timestamp}.png')
                depth_image_path = os.path.join(date_dir, f'depth_{timestamp}.png')

                # Save the color image
                cv2.imwrite(rgb_image_path, color_image)

                # Save the depth image as a PNG file
                cv2.imwrite(depth_image_path, depth_image_8bit)

                last_capture_time = current_time  # Update the last capture time

            # Display frames while recording
            if DISPLAY_FRAMES:
                cv2.imshow('RGB Frame', color_image)
                cv2.imshow('Depth Frame', depth_image_8bit)

            # Check if 'q' is pressed to exit early
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        # Stop the pipeline and release resources
        pipeline.stop()
        cv2.destroyAllWindows()
