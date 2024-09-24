import pyrealsense2 as rs
import os, yaml, math


class D456:

    def __init__(self, ID="rs_default", calibrate=False):
        self.__id = ID
        self.__pipeline = rs.pipeline()
        self.__config = rs.config()
        self.__calibrate = calibrate

        params = self.load_camera_params(self.__id)
        self.__imu_calibration_coefficients = self.setup(params)
        self.__pipeline.start(self.__config)

        if calibrate:
            params = self.load_camera_params(self.__id, self.__calibrate)
            self.__imu_calibration_coefficients = self.setup(params)
            self.__pipeline.stop()
            self.__pipeline.start(self.__config)
        

    def load_camera_params(self, ID, calibrate=False):

        config_params = None
        params = None

        try:
            # Read the params file
            config_path = os.path.join(os.getcwd(),"config", "camera_params.yaml")
            print("Attempting to open " + str(config_path) + "...")
            with open(config_path, 'r+') as file:
                params = yaml.safe_load(file)
                print("Loading camera parameters...\n")

                for camera_name, camera in params['cameras'].items():
                    # print(camera['id'])
                    if camera.get('id') == ID:
                        config_params = camera                        
                
                        print("Camera ID match found!")
                        print("**** Configuration Parameters ****")
                        print(config_params)
                        print("**** END PARAMS ****")
                        break

                if config_params == None:
                    config_params = params['cameras'].get('camera_0', None)
                    print(f"Unable to locate configuration for {ID}. Loading default params")

                file.close()
        
            if calibrate:
                input("Place the camera on a stable, flat surface.  Press <ENTER> when ready to zero...")
                print("Zeroing...")

                while True:
                    # Wait for a coherent pair of frames: accelerometer and gyroscope
                    print("Waiting for coherent pair of frames from accelerometer and gyro")
                    frames = self.__pipeline.wait_for_frames()

                    accel_frame = frames.first_or_default(rs.stream.accel)
                    gyro_frame = frames.first_or_default(rs.stream.gyro)

                    if accel_frame and gyro_frame:
                        # Extract data from accelerometer
                        accel_data = accel_frame.as_motion_frame().get_motion_data()
                        print(f"Accelerometer (X,Y,Z): {accel_data.x}, {accel_data.y}, {accel_data.z}")

                        # Extract data from gyroscope
                        gyro_data = gyro_frame.as_motion_frame().get_motion_data()
                        print(f"Gyroscope (X,Y,Z): {gyro_data.x}, {gyro_data.y}, {gyro_data.z}")
                        break
                
                # Open the file after the loop
                with open(config_path, 'w+') as file:
                    #iterate over the items in the original params until we find the parameters for the camera of interest

                    for camera_name, camera in params['cameras'].items():
                        # print(camera['id'])
                        if camera.get('id') == self.__id:
                            print("Camera ID match found! Writing calibration data...")

                            # replace the imu offsets in the dictionary                           
                            
                            imu_offsets = camera.get('imu_offsets')
                            imu_offsets['x'] = accel_data.x
                            imu_offsets['y'] = accel_data.y
                            imu_offsets['z'] = accel_data.z
                            
                            config_params = camera                        
                            
                            print("**** New Configuration Parameters ****")
                            print(config_params)
                            print("**** END PARAMS ****")
                            break

                    yaml.dump(params,file, default_flow_style=False)

                    if config_params is None:
                        config_params = params['cameras'].get('camera_0', None)
                        print(f"Unable to locate configuration for {ID}. Loading default params")
        
        except FileNotFoundError:
            print(f"File {config_path} not found!")

        finally:
            return config_params

    def setup(self, params):
        self.__width = params['resolution'].get('width')
        self.__height = params['resolution'].get('height')
        self.__frame_rate = int(params['frame_rate'])
        
        imu_offsets = params['imu_offsets']
        coefficients = (imu_offsets['x'], imu_offsets['y'], imu_offsets['z'],
                                               imu_offsets['roll'], imu_offsets['pitch'], imu_offsets['yaw'])
        
        streams = params['enabled_streams']

        if streams.get('color_stream') == 1:
            print("Color stream enabled")
            self.__config.enable_stream(rs.stream.color, self.__width, self.__height, rs.format.bgr8, self.__frame_rate)

        if streams.get('depth_stream') == 1:
            print("Depth stream enabled")
            self.__config.enable_stream(rs.stream.depth, self.__width, self.__height, rs.format.z16, self.__frame_rate)

        if streams.get('ir_stream') == 1:
            print("IR stream enabled")
            self.__config.enable_stream(rs.stream.infrared, self.__width, self.__height, rs.format.y8, self.__frame_rate)
        
        if streams.get('accel_stream') == 1:
            print("IMU stream enabled")
            self.__config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f,100)  # 100 Hz
            self.__config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)   # 200 Hz

        return coefficients
        
        # print(f"{self.__id} resolution is {self.__width} x {self.__height}")

    def get_config(self):
        return self.__config

    def get_pose(self):
        pass
    
    def get_x_offset(self):
        return self.__imu_calibration_coefficients[0]
    
    def get_y_offset(self):
        return self.__imu_calibration_coefficients[1]
    
    def get_z_offset(self):
        return self.__imu_calibration_coefficients[2]
    
    def get_roll_offset(self):
        return self.__imu_calibration_coefficients[3]
    
    def get_pitch_offset(self):
        return self.__imu_calibration_coefficients[4]
    
    def get_yaw_offset(self):
        return self.__imu_calibration_coefficients[5]

    def stop(self):
        self.__pipeline.stop()

    def start(self):
        self.__pipeline.start()

    def get_pitch_angle(self):

        while True:

            frames = self.__pipeline.wait_for_frames()
            accel_frame = frames.first_or_default(rs.stream.accel)
            gyro_frame = frames.first_or_default(rs.stream.gyro)

            if accel_frame and gyro_frame:
                accel_data = accel_frame.as_motion_frame().get_motion_data()
                # print(f"Accelerometer (X,Y,Z): {accel_data.x}, {accel_data.y}, {accel_data.z}")

            break
        
        a_y = accel_data.y - self.get_y_offset()
        a_z = accel_data.z - self.get_z_offset()
        
        try:
            
            pitch = math.degrees(math.atan(a_y/a_z)) - 90

        except ValueError:
            if abs(a_y) > 9.81:
                pitch = 0

        finally:
            print(f"Pitch Angle: {round(pitch,3)} degrees")


if __name__ == '__main__':
    camera = D456(ID="d456_002")
    camera.get_pitch_angle()

