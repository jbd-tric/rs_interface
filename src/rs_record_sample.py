import pyrealsense2 as rs
import numpy as np
import cv2, time, os
from datetime import datetime

# def parse_arguments():
#     parser = argparse.ArgumentParser(description='Capture RGB and Depth ')


if __name__ == '__main__':
    
    # Constants
    RESOLUTION_WIDTH = 1280
    RESOLUTION_HEIGHT = 720
    FRAME_RATE = 30 # Max supported frame rate is 30 FPS
    DISPLAY_FRAMES = False
    RECORD_DURATION = 5 # seconds

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

    # Create the OpenCV VideoWriter objects for saving RGB and Depth streams
    fourcc = cv2.VideoWriter_fourcc(*'mp4v') # Codec
    timestamp = str(int(time.time()))  # UNIX time as a suffix

    rgb_out_path = os.path.join(date_dir, f'rgb_out_{timestamp}.mp4')
    depth_out_path = os.path.join(date_dir, f'depth_out_{timestamp}.mp4')

    rgb_out = cv2.VideoWriter(rgb_out_path, fourcc, FRAME_RATE, (RESOLUTION_WIDTH, RESOLUTION_HEIGHT))
    depth_out = cv2.VideoWriter(depth_out_path, fourcc, FRAME_RATE, (RESOLUTION_WIDTH, RESOLUTION_HEIGHT))

    # Start recording
    start_time = time.time()

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

            # Write frames to video files
            rgb_out.write(color_image)
            depth_out.write(cv2.cvtColor(depth_image_8bit, cv2.COLOR_GRAY2BGR)) # Convert to BGR for video
            
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
        rgb_out.release()
        depth_out.release()
        cv2.destroyAllWindows()
