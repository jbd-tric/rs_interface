import pyrealsense2 as rs
import numpy as np
import cv2, time

if __name__ == '__main__':
    
    # Constants
    RESOLUTION_WIDTH = 640
    RESOLUTION_HEIGHT = 480
    FRAME_RATE = 30 # Max supported frame rate is 30 FPS
    DISPLAY_FRAMES = True
    RECORD_DURATION = 30 #seconds

    # Initialize RealSense Pipeline
    pipeline = rs.pipeline()
    config = rs.config()

    # Enable color and depth streams
    config.enable_stream(rs.stream.color, RESOLUTION_WIDTH, RESOLUTION_HEIGHT, rs.format.bgr8, FRAME_RATE)
    config.enable_stream(rs.stream.depth, RESOLUTION_WIDTH, RESOLUTION_HEIGHT, rs.format.z16, FRAME_RATE)

    # start the pipeline
    pipeline.start(config)

    # get the depth scale to convert depth data to meters
    depth_sensor = pipeline.get_active_profile().get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()

    # Create the OpenCV VideoWriter objects for saving RGB and Depth streams
    fourcc = cv2.VideoWriter_fourcc(*'mp4v') # Codec
    rgb_out = cv2.VideoWriter('rgb_out.mp4', fourcc, FRAME_RATE, (RESOLUTION_WIDTH, RESOLUTION_HEIGHT))
    depth_out = cv2.VideoWriter('depth_out.mp4', fourcc, FRAME_RATE, (RESOLUTION_WIDTH, RESOLUTION_HEIGHT))

    # Start recording
    start_time = time.time()

    try:
        while time.time() - start_time < RECORD_DURATION:
            # wait for a new set of frames
            frames = pipeline.wait_for_frames()

            # get the rgb frame
            color_frame = frames.get_color_frame()

            # get the depth frame
            depth_frame = frames.get_depth_frame()

            if not color_frame or not depth_frame:
                continue

            # convert images to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            # convert depth image to 8-bit for saving
            depth_image_8bit = cv2.convertScaleAbs(depth_image, alpha=0.03)

            rgb_out.write(color_image)
            depth_out.write(cv2.cvtColor(depth_image_8bit, cv2.COLOR_GRAY2BGR)) # Convert to BGR for video
            
            # display frames while recording
            if DISPLAY_FRAMES:
                cv2.imshow('RGB Frame', color_image)
                cv2.imshow('Depth Frame', depth_image_8bit)
                #cv2.imshow('Depth_Frame 16 bit', depth_image)

            # Check if 'q' is pressed to exit early
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        # stop the pipeline and release resources
        pipeline.stop()
        rgb_out.release()
        depth_out.release()
        cv2.destroyAllWindows()