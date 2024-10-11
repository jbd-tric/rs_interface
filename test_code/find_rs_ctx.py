import pyrealsense2 as rs
import re
import subprocess


# Find all RealSense Devices attached to the bus
def find_rs_devices(target_model=""):
    rs_devices = rs.context().query_devices()
                
    return rs_devices

if __name__ == '__main__':
    rs_devices = find_rs_devices()
    
    for device in rs_devices:
        # device.group_dict()
        print(device)
    