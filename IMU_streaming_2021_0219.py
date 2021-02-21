import serial
import time
import struct
from multiprocessing import Process
import os

## comment the import lines below if no figure needed ##
# import matplotlib.pyplot as plt
# from matplotlib.animation import FuncAnimation

def run_IMU_streaming():

    g = 9.81
    convL = 16.0 / 32768.0 * g  # m/s**2
    convAv = 2000.0 / 32768.0  # deg/s
    convA = 180.0 / 32768  # deg
    convM = 1.0  #

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

            bytes = s_conn.read(88)  # do we need to make this 88?

            acc_index = bytes.find(b'\x51') + 1
            angVel_index = bytes.find(b'\x52') + 1
            ang_index = bytes.find(b'\x53') + 1
            mag_index = bytes.find(b'\x54') + 1

            axesLinAcc = struct.unpack("<hhh", bytes[acc_index:acc_index + 6])
            Ax, Ay, Az = [a * convL for a in axesLinAcc]
            axesAngVel = struct.unpack("<hhh", bytes[angVel_index:angVel_index + 6])
            Gx, Gy, Gz = [a * convAv for a in axesAngVel]
            axesMag = struct.unpack("<hhh", bytes[mag_index:mag_index + 6])
            Mx, My, Mz = [a * convM for a in axesMag]
            axesAng = struct.unpack("<hhh", bytes[ang_index:ang_index + 6])
            Roll, Pitch, Yaw = [a * convA for a in axesAng]
            time_diff = time.time() - time_start

            time_list.append(time_diff)
            print("%6d, %10.3f" % (num, (sum(time_list) / len(time_list)) * 1000))

            imu_reading = [time_diff, Roll, Pitch, Yaw, Gx, Gy, Gz, Ax, Ay, Az, Mx, My, Mz]
            file1.writelines(
                ','.join(str(j) for j in imu_reading) + '\n')

            # print(Roll, Pitch, Yaw, Gx, Gy, Gz, Ax, Ay, Az, Mx, My, Mz)
            num += 1

            return imu_reading

    except KeyboardInterrupt:

        s_conn.close()
        file1.close()
        print('Program Interrupted! Closing...')


if __name__ == '__main__':
    print('Parent process %s.' % os.getpid())
    p = Process(target=run_IMU_streaming, args=())
    print('Child process will start.')
    p.start()
    p.join()
    print('Child process end.')


