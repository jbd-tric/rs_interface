import re
import subprocess


# Find all RealSense Devices attached to the bus
def find_rs_devices(target_model=""):
    rs_regex = re.compile("Bus\s+(?P<bus>\d+)\s+Device\s+(?P<device>\d+).+ID\s(?P<tag>.+)$",re.I)
    df = subprocess.check_output("lsusb")   # executes the 'lsusb' command in the terminal and saves the output as a string

    rs_devices = []
    rs_flag = "Intel Corp. Intel(R) RealSense(TM) Depth Module " + target_model

    for i in df.split(b'\n'):   # create a list object, splitting on the newline character
        if i:
            # print(i)
            info = rs_regex.match(i.decode("utf-8")) # cast each bytes object to a string, match to regex
            
            if info:
                dinfo = info.groupdict()
                # print(dinfo)
                
                if dinfo['tag'].__contains__(rs_flag):
                    rs_devices.append(dinfo)
                
                # dinfo['device'] = 'dev/bus/usb/%s/%s' % (dinfo.pop('bus'), dinfo.pop('device'))
                
    return rs_devices

if __name__ == '__main__':
    rs_devices = find_rs_devices()
    print(f"Found {len(rs_devices)} RealSense devices on bus:\n")
    for device in rs_devices: print(device)