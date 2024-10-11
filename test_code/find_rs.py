import re
import subprocess

def find_rs_devices():
    rs_regex = re.compile("Bus\s+(?P<bus>\d+)\s+Device\s+(?P<device>\d+).+ID\s(?P<tag>.+)$",re.I)
    df = subprocess.check_output("lsusb")

    devices = []

    for i in df.split(b'\n'):
        if i:
            info = rs_regex.match(i)
            if info:
                dinfo = info.groupdict()
                dinfo['device'] = 'dev/bus/usb/%s/%s' % (dinfo.pop('bus'), dinfo.pop('device'))
                devices.append(dinfo)

    print(devices)

if __name__ == '__main__':
    find_rs_devices()