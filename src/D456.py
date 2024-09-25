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
    ## @param id Unique camera ID for identification. Default is "rs_default".
    ## @param calibrate Boolean flag to determine whether to perform camera calibration. Default is False.
    def __init__(self, id="rs_default", calibrate=False):
        self.__id = id  # Store the camera ID
        self.__pipeline = rs.pipeline()  # Initialize the camera pipeline
        self.__config = rs.config()  # Camera configuration object
        self.__calibrate = calibrate  # Calibration flag
        self.__enabled_streams = []  # List to track enabled streams

        # Load camera parameters from YAML file
        params = self.load_camera_params(self.__id)
        # Store IMU calibration coefficients for the camera
        self.__imu_calibration_coefficients = self.setup(params)
        self.__pipeline.start(self.__config)  # Start the camera pipeline with the config

        # If calibration is requested, perform calibration and restart the pipeline
        if calibrate:
            params = self.load_camera_params(self.__id, self.__calibrate)
            self.__imu_calibration_coefficients = self.setup(params)
            self.__pipeline.stop()
            self.__pipeline.start(self.__config)
        
    ## @brief Loads the camera parameters from the YAML file and optionally performs calibration.
    ## 
    ## @param ID Unique camera ID for identification.
    ## @param calibrate Boolean flag to determine if calibration should be performed.
    ## @return Dictionary containing configuration parameters for the specified camera.
    def load_camera_params(self, ID, calibrate=False):
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
                    if camera.get('id') == ID:
                        config_params = camera  # Store matching configuration
                        print("Camera ID match found!")
                        print("**** Configuration Parameters ****")
                        print(config_params)
                        print("**** END PARAMS ****")
                        break

                # If no matching camera ID is found, use default camera_0 params
                if config_params == None:
                    config_params = params['cameras'].get('camera_0', None)
                    print(f"Unable to locate configuration for {ID}. Loading default params")
                file.close()
        
            # Perform calibration if specified
            if calibrate:
                input("Place the camera on a stable, flat surface. Press <ENTER> when ready to zero...")
                print("Zeroing...")

                # Loop to ensure coherent frames are captured for calibration
                while True:
                    print("Waiting for coherent pair of frames from accelerometer and gyro")
                    frames = self.__pipeline.wait_for_frames()

                    # Extract motion data from accelerometer and gyroscope frames
                    accel_frame = frames.first_or_default(rs.stream.accel)
                    gyro_frame = frames.first_or_default(rs.stream.gyro)

                    if accel_frame and gyro_frame:
                        # Get accelerometer and gyroscope data
                        accel_data = accel_frame.as_motion_frame().get_motion_data()
                        print(f"Accelerometer (X,Y,Z): {accel_data.x}, {accel_data.y}, {accel_data.z}")
                        gyro_data = gyro_frame.as_motion_frame().get_motion_data()
                        print(f"Gyroscope (X,Y,Z): {gyro_data.x}, {gyro_data.y}, {gyro_data.z}")
                        break
                
                # Write calibration data back to the YAML file
                with open(config_path, 'w+') as file:
                    for camera_name, camera in params['cameras'].items():
                        if camera.get('id') == self.__id:
                            print("Camera ID match found! Writing calibration data...")

                            # Update IMU offsets with calibration data
                            imu_offsets = camera.get('imu_offsets')
                            imu_offsets['x'] = accel_data.x
                            imu_offsets['y'] = accel_data.y
                            imu_offsets['z'] = accel_data.z
                            
                            config_params = camera  # Update config params
                            
                            print("**** New Configuration Parameters ****")
                            print(config_params)
                            print("**** END PARAMS ****")
                            break

                    # Save updated parameters back to the file
                    yaml.dump(params, file, default_flow_style=False)

                    if config_params is None:
                        config_params = params['cameras'].get('camera_0', None)
                        print(f"Unable to locate configuration for {ID}. Loading default params")
        
        except FileNotFoundError:
            print(f"File {config_path} not found!")

        finally:
            return config_params

    ## @brief Sets up the camera's configuration parameters and enables streams.
    ## 
    ## @param params Dictionary containing camera parameters such as resolution, frame rate, and enabled streams.
    ## @return Tuple of IMU calibration coefficients (x, y, z, roll, pitch, yaw).
    def setup(self, params):
        self.__width = params['resolution'].get('width')
        self.__height = params['resolution'].get('height')
        self.__frame_rate = int(params['frame_rate'])
        self.__accel_rate = int(params['accel_rate'])
        self.__gyro_rate = int(params['gyro_rate'])
        
        # Extract IMU offsets
        imu_offsets = params['imu_offsets']
        coefficients = (imu_offsets['x'], imu_offsets['y'], imu_offsets['z'],
                                               imu_offsets['roll'], imu_offsets['pitch'], imu_offsets['yaw'])
        
        # Enable the specified streams
        streams = params['enabled_streams']

        if streams.get('color_stream'):
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

        return coefficients

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

    ## @brief Returns the camera's frame rate.
    ## @return Frame rate as an integer.
    def get_frame_rate(self):
        return self.__frame_rate
    
    ## @brief Returns the camera's accelerometer rate.
    ## @return Accelerometer rate as an integer.
    def get_accel_rate(self):
        return self.__accel_rate
    
    ## @brief Returns the camera's gyroscope rate.
    ## @return Gyroscope rate as an integer.
    def get_gyro_rate(self):
        return self.__gyro_rate

    ## @brief Retrieves pose data from the IMU, applying the calibrated offsets.
    ## @return Dictionary containing acceleration and angular velocity.
    def get_pose(self):
        accel_data = None
        gyro_data = None
        
        while True:
            frames = self.__pipeline.wait_for_frames()  # Wait for frames from the camera
            
            # Extract accelerometer and gyroscope frames
            accel_frame = frames.first_or_default(rs.stream.accel)
            gyro_frame = frames.first_or_default(rs.stream.gyro)

            if accel_frame and gyro_frame:
                # Extract motion data
                accel_data = accel_frame.as_motion_frame().get_motion_data()
                gyro_data = gyro_frame.as_motion_frame().get_motion_data()
                break

        # Apply IMU calibration offsets to the raw data
        a_x = accel_data.x - self.get_x_offset()
        a_y = accel_data.y - self.get_y_offset()
        a_z = accel_data.z - self.get_z_offset()

        g_roll = gyro_data.x - self.get_roll_offset()
        g_pitch = gyro_data.y - self.get_pitch_offset()
        g_yaw = gyro_data.z - self.get_yaw_offset()

        # Create a pose dictionary with acceleration and angular velocity
        pose = {
            'acceleration': {
                'x': a_x,
                'y': a_y,
                'z': a_z
            },
            'angular_velocity': {
                'roll': g_roll,
                'pitch': g_pitch,
                'yaw': g_yaw
            }
        }

        print(f"Pose data:\nAcceleration (X,Y,Z): {a_x}, {a_y}, {a_z}\n"
            f"Angular velocity (roll, pitch, yaw): {g_roll}, {g_pitch}, {g_yaw}")

        return pose
    
    ## @brief Gets the X-axis offset from IMU calibration.
    ## @return X-axis offset.
    def get_x_offset(self):
        return self.__imu_calibration_coefficients[0]
    
    ## @brief Gets the Y-axis offset from IMU calibration.
    ## @return Y-axis offset.
    def get_y_offset(self):
        return self.__imu_calibration_coefficients[1]
    
    ## @brief Gets the Z-axis offset from IMU calibration.
    ## @return Z-axis offset.
    def get_z_offset(self):
        return self.__imu_calibration_coefficients[2]
    
    ## @brief Gets the roll offset from IMU calibration.
    ## @return Roll offset.
    def get_roll_offset(self):
        return self.__imu_calibration_coefficients[3]
    
    ## @brief Gets the pitch offset from IMU calibration.
    ## @return Pitch offset.
    def get_pitch_offset(self):
        return self.__imu_calibration_coefficients[4]
    
    ## @brief Gets the yaw offset from IMU calibration.
    ## @return Yaw offset.
    def get_yaw_offset(self):
        return self.__imu_calibration_coefficients[5]

    ## @brief Stops the camera pipeline.
    def stop(self):
        self.__pipeline.stop()

    ## @brief Starts the camera pipeline.
    def start(self):
        self.__pipeline.start()

    ## @brief Calculates the camera's pitch angle using accelerometer data.
    ## @return None
    def get_pitch_angle(self):

        while True:
            frames = self.__pipeline.wait_for_frames()
            accel_frame = frames.first_or_default(rs.stream.accel)
            gyro_frame = frames.first_or_default(rs.stream.gyro)

            if accel_frame and gyro_frame:
                # Get accelerometer data
                accel_data = accel_frame.as_motion_frame().get_motion_data()
                break
        
        # Apply Y and Z offsets to the accelerometer data
        a_y = accel_data.y - self.get_y_offset()
        a_z = accel_data.z - self.get_z_offset()
        
        try:
            # Calculate the pitch angle using the arctangent of the accelerometer data
            pitch = math.degrees(math.atan(a_y/a_z)) - 90

        except ValueError:
            if abs(a_y) > 9.81:  # Check if acceleration exceeds gravity's force
                pitch = 0

        finally:
            print(f"Pitch Angle: {round(pitch, 3)} degrees")

if __name__ == '__main__':
    camera = D456(ID="d456_001")  # Initialize the camera with ID
    camera.get_pitch_angle()  # Get the pitch angle
