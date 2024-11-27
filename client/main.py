from time import time

import serial

ser = serial.Serial(port="/dev/ttyUSB0", baudrate=500000)
t_last, a_z_last = time(), float(ser.readline().decode().split()[2])
dirn = 1
cycle_start_time = 0
calibration_factor = 10

while True:
    a_x, a_y, a_z, r_x, r_y, r_z = ser.readline().decode().split()
    t = time()

    a_z = float(a_z)
    slope = (a_z - a_z_last) / (t - t_last)


    if dirn == 1 and slope < 0:
        cycle_start_time = t
        dirn = 0
    elif dirn == 0 and slope > 0:
        dirn = 1
        print(f"RPM: {int(calibration_factor / (t - cycle_start_time))}")
