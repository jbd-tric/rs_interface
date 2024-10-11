import pyrealsense2 as rs
import os, yaml, math
import numpy as np

## @file D456.py
## @brief Class to interface with Intel RealSense D456 Camera.
## 
## This class initializes a RealSense D456 camera, loads camera parameters from a YAML configuration file, 
## and provides functionality for camera calibration, streaming data (RGB, depth, IR, and IMU), and extracting motion data.
## 
## @author Jason Davis
## @date 9/24/2024
## @license GPL 3.0

class D456:
    ## @brief Initializes the D456 class.
    ## 
    ## @param serial Manufacturer-assigned serial number for identification. Default is "unspecified".
    def __init__(self, serial):
        self.__serial = serial  # Store the camera serial number
        self.__pipeline = rs.pipeline()  # Initialize the camera pipeline
        self.__config = rs.config()  # Camera configuration object
        self.__calibrate = calibrate  # Calibration flag
        self.__enabled_streams = []  # List to track enabled streams

        # Load camera parameters from YAML file
        params = self.load_camera_params(self.__serial)
        
    ## @brief Loads the camera parameters from the YAML file and optionally performs calibration.
    ## 
    ## @param ID Unique camera ID for identification.
    ## @return Dictionary containing configuration parameters for the specified camera.
    def load_camera_params(self, serial):
        config_params = None
        params = None

        try:
            # Get the configuration file path for camera parameters
            config_path = os.path.join(os.path.dirname(os.getcwd()), "config", "camera_params.yaml")
            print("Attempting to open " + str(config_path) + "...")
            
            # Load camera parameters from the YAML file
            with open(config_path, 'r+') as file:
                params = yaml.safe_load(file)
                print("Loading camera parameters...\n")

                # Search for the matching camera ID in the parameters file
                for camera_name, camera in params['cameras'].items():
                    if camera.get('serial_number') == serial:
                        config_params = camera  # Store matching configuration
                        print("Camera ID match found!")
                        print("**** Configuration Parameters ****")
                        print(config_params)
                        print("**** END PARAMS ****")
                        break

                # If no matching camera ID is found, use default camera_0 params
                if config_params == None:
                    config_params = params['cameras'].get('camera_default', None)
                    print(f"Unable to locate configuration for {serial}. Loading default params")
                file.close()
        
        except FileNotFoundError:
            print(f"File {config_path} not found!")

        finally:
            return config_params

    ## @brief Sets up the camera's configuration parameters and enables streams.
    ## 
    ## @param params Dictionary containing camera parameters such as resolution, frame rate, and enabled streams.
    
    def setup(self, params):
        
        # Enable the specified streams
        streams = []

        if params['streams']:
            print("Color stream enabled")
            self.__config.enable_stream(rs.stream.color, self.__width, self.__height, rs.format.bgr8, self.__frame_rate)
            self.__enabled_streams.append('rgb_stream')

        if streams.get('depth_stream'):
            print("Depth stream enabled")
            self.__config.enable_stream(rs.stream.depth, self.__width, self.__height, rs.format.z16, self.__frame_rate)
            self.__enabled_streams.append('depth_stream')

        if streams.get('ir_stream'):
            print("IR stream enabled")
            self.__config.enable_stream(rs.stream.infrared, self.__width, self.__height, rs.format.y8, self.__frame_rate)
            self.__enabled_streams.append('ir_stream')

        if streams.get('accel_stream') == 1:
            print("IMU stream enabled")
            self.__config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 200)  # 100 Hz
            self.__config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)   # 200 Hz
            self.__enabled_streams.append('imu_stream')

    ## @brief Returns the camera's ID.
    ## @return Camera ID as a string.
    def get_id(self):
        return self.__id

    ## @brief Returns the current configuration object.
    ## @return rs.config object.
    def get_config(self):
        return self.__config
    
    ## @brief Returns the camera's pipeline.
    ## @return rs.pipeline object.
    def get_pipeline(self):
        return self.__pipeline

    ## @brief Returns the list of enabled streams.
    ## @return List of enabled stream names.
    def get_enabled_streams(self):
        return self.__enabled_streams
    
    ## @brief Captures and returns an RGB and depth image pair from the camera.
    ## @return Tuple containing a color image (RGB) and depth image as NumPy arrays.
    def get_rgbd_image_pair(self):
        align = rs.align(rs.stream.color)  # Align depth frames to the color stream
        frames = self.__pipeline.wait_for_frames()  # Wait for frames
        aligned_frames = align.process(frames)

        # Get aligned color and depth frames
        color_frame = aligned_frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        return (color_image, depth_image)

    ## @brief Stops the camera pipeline.
    def stop(self):
        self.__pipeline.stop()

    ## @brief Starts the camera pipeline.
    def start(self):
        self.__pipeline.start(self.__config)

if __name__ == '__main__':
    camera = D456(ID="d456_001")  # Initialize the camera with ID
    camera.get_pitch_angle()  # Get the pitch angle
