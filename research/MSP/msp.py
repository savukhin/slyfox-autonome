import sys
# from msp.multiwii import MultiWii
from pymultiwii import MultiWii

# if __name__ == "__main__":
#     try:
#         print_debug = sys.argv[1].lower() == 'true'
#         fc = MultiWii("/dev/ttyS0", print_debug)
#         fc.start()
#         while True:
#             print(fc.get_attitude())

#     except Exception as error:
#         import traceback
#         print("Error on Main: " + str(error))
#         traceback.print_exc()


serialPort = "COM8"
# serialPort = "/dev/tty.usbserial-A101CCVF"
board = MultiWii(serialPort)
# print(board.arm())
# print(board.sendCMD(8, MultiWii.MOTOR, [1500, 1000, 1000, 1000], 'H2B'))
while True:
    print(board.getData(MultiWii.RAW_IMU))
    # print(board.getData(MultiWii.ATTITUDE))
    # print(board.getData(MultiWii.ATTITUDE))
    # board.sendCMD()


# import re
# import subprocess
# device_re = re.compile("Bus\s+(?P<bus>\d+)\s+Device\s+(?P<device>\d+).+ID\s(?P<id>\w+:\w+)\s(?P<tag>.+)$", re.I)
# df = subprocess.check_output("lsusb")
# devices = []
# for i in df.split('\n'):
#     if i:
#         info = device_re.match(i)
#         if info:
#             dinfo = info.groupdict()
#             dinfo['device'] = '/dev/bus/usb/%s/%s' % (dinfo.pop('bus'), dinfo.pop('device'))
#             devices.append(dinfo)
# print(devices)