'''
This file is for LP Research IMU / Blue IMU
'''


import serial
import time
import struct
from multiprocessing import Process
import os
import numpy as np

## comment the import lines below if no figure needed ##
# import matplotlib.pyplot as plt
# from matplotlib.animation import FuncAnimation

def run_IMU_streaming():

    g = 9.81
    rad2deg = 180 / np.pi
    # convL = 16.0 / 32768.0 * g  # m/s**2
    # convAv = 2000.0 / 32768.0  # deg/s
    # convA = 180.0 / 32768  # deg
    # convM = 1.0  #

    serialPort = '/dev/rfcomm0'  # /dev/ttyUSB0 # change this line if bluetooth or other port
    # serialPort = '/dev/ttyUSB0'
    serialBaud = 115200

    # start = time.time()
    timestr = time.strftime("%Y%m%d-%H%M%S")
    file_name = "Data/" + "data-" + timestr + ".txt"
    file1 = open(file_name, "w")
    # file1.write("Roll,Pitch,Yaw,Gx,Gy,Gz, Ax,Ay,Az,,Mx,My,Mz,\n")


    # Connect to serial port
    print('Trying to connect to ' + str(serialPort) +
          ' at ' + str(serialBaud) + ' BAUD.')
    try:
        s_conn = serial.Serial(serialPort, serialBaud, timeout=None)
        print('Connected!')
    except:
        print("Failed to connect with " + str(serialPort) +
              ' at ' + str(serialBaud) + ' BAUD.')



    try:
        print("Starting serial data transfer, press Ctrl+C to stop.")
        time_list = []
        num = 0
        while True:
            time_start = time.time()

            bytes = s_conn.read(39)  # Each package has 39 bytes

            start_index = bytes.find(b'\x3A\x01\x00')
            end_index = bytes.find(b'\x0D\x0A', start_index + 1)

            time_index = start_index + 7
            mag_index = start_index + 11
            ang_vel_index = start_index + 17
            angle_index = start_index + 23
            acc_index = start_index + 29

            time_bytes = bytes[time_index:time_index + 4]
            time_imu = struct.unpack("<I", time_bytes)[0]/400

            mag = struct.unpack("<hhh", bytes[mag_index:mag_index + 6])
            mag_x, mag_y, mag_z = [a / 100 for a in mag]

            ang_vel = struct.unpack("<hhh", bytes[ang_vel_index:ang_vel_index + 6])
            ang_vel_x, ang_vel_y, ang_vel_z = [a / 1000 * rad2deg for a in ang_vel]

            angle = struct.unpack("<hhh", bytes[angle_index:angle_index + 6])
            ang_x, ang_y, ang_z = [a / 10000 * rad2deg for a in angle]

            acc = struct.unpack("<hhh", bytes[acc_index:acc_index + 6])
            acc_x, acc_y, acc_z = [a / 1000 * g for a in acc]

            time_diff = time.time() - time_start

            # if num > 100:
            #     time_list.append(time_diff)
            #     print("%6d, %10.3f" % (num, (sum(time_list) / len(time_list)) * 1000))

            # imu_reading = [time_diff, Roll, Pitch, Yaw, Gx, Gy, Gz, Ax, Ay, Az, Mx, My, Mz]
            imu_reading = [time_diff, ang_x, ang_y, ang_z, ang_vel_x, ang_vel_y, ang_vel_z, acc_x, acc_y, acc_z, mag_x, mag_y, mag_z]
            file1.writelines(','.join(str(j) for j in imu_reading) + '\n')

            # print(num, Roll, Pitch, Yaw, Gx, Gy, Gz, Ax, Ay, Az, Mx, My, Mz)
            print("%9.4f, %9.4f, %9.4f, %9.4f, %9.4f, %9.4f, %9.4f, %9.4f, %9.4f, %9.4f, %9.4f, %9.4f, %9.4f" %
                  (time_imu, mag_x, mag_y, mag_z, ang_vel_x, ang_vel_y, ang_vel_z, ang_x, ang_y, ang_z, acc_x, acc_y, acc_z))
            num += 1

            # return imu_reading

    except KeyboardInterrupt:

        s_conn.close()
        file1.close()
        print('Program Interrupted! Closing...')


if __name__ == '__main__':
    print('Parent process %s.' % os.getpid())
    run_IMU_streaming()
    # p = Process(target=run_IMU_streaming, args=())
    # print('Child process will start.')
    # p.start()
    # p.join()
    print('Child process end.')


