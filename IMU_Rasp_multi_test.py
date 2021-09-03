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

    # serialPort = '/dev/rfcomm0'  # /dev/ttyUSB0 # change this line if bluetooth or other port
    # serialPort = '/dev/ttyUSB0'
    serialPort = '/dev/ttyACM0'
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
            # Header: 3 bytes; Each IMU has 56 bytes; Up to seven IMUs
            # Each IMU: time * 1, angle_vel * 3, angle * 3, lin acc * 3, quaternion * 4
            # All data in float
            bytes = s_conn.read(399)
            # if time.time() - time_start > 0.0:
                # print(bytes)

            start_index = bytes.find(b'\x3A\x88')
            if start_index != 0:
                bytes_ignore = s_conn.read(start_index)
                # print(bytes_ignore)
            else:
                end_index = bytes.find(b'\x0D\x0A', start_index + 1)
                # print(start_index, end_index)

                # Up to seven IMUs
                imu_reading = []
                for num_imu in range(5):
                    time_index = start_index + 3 + num_imu * 56
                    ang_vel_index = start_index + 7 + num_imu * 56
                    angle_index = start_index + 19 + num_imu * 56
                    acc_index = start_index + 31 + num_imu * 56
                    quaternion_index = start_index + 43 + num_imu * 56
                    # print(time_index)

                    time_bytes = bytes[time_index:time_index + 4]
                    time_imu = struct.unpack("<f", time_bytes)[0]

                    quaternion = struct.unpack("<ffff", bytes[quaternion_index:quaternion_index + 16])
                    quat_1, quat_2, quat_3, quat_4 = [a for a in quaternion]

                    ang_vel = struct.unpack("<fff", bytes[ang_vel_index:ang_vel_index + 12])
                    ang_vel_x, ang_vel_y, ang_vel_z = [a for a in ang_vel]

                    angle = struct.unpack("<fff", bytes[angle_index:angle_index + 12])
                    ang_x, ang_y, ang_z = [a for a in angle]

                    acc = struct.unpack("<fff", bytes[acc_index:acc_index + 12])
                    acc_x, acc_y, acc_z = [a for a in acc]

                    time_diff = time.time() - time_start

                    one_imu_reading = [time_diff, ang_x, ang_y, ang_z, ang_vel_x, ang_vel_y, ang_vel_z,
                                       acc_x, acc_y, acc_z, quat_1, quat_2, quat_3, quat_4]
                    imu_reading.extend(one_imu_reading)

                    print("%9d, %9.4f, %9.4f, %9.4f, %9.4f, %9.4f, %9.4f, %9.4f, %9.4f, %9.4f, %9.4f" %
                          (num_imu, time_imu, ang_x, ang_y, ang_z, ang_vel_x, ang_vel_y, ang_vel_z, acc_x, acc_y, acc_z))

                file1.writelines(','.join(str(j) for j in imu_reading) + '\n')

            num += 1


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


