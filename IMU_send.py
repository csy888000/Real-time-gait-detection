import serial
import time
import struct

g = 9.81
convL = 16.0 / 32768.0 * g  # m/s**2
convAv = 2000.0 / 32768.0  # deg/s
convA = 180.0 / 32768  # deg
convM = 1.0  #
time_list = []
# num = 0

def receive_from_IMU(serial_conn, num):

    time_start = time.time()

    bytes = serial_conn.read(88)  # do we need to make this 88?

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
    time_diff = (time.time() - time_start) * 1000

    # if num > 0:
    #     time_list.append(time_diff)
    #     print("%6d, %10.3f" % (num, (sum(time_list) / len(time_list))))

    imu_reading = [Roll, Pitch, Yaw, Gx, Gy, Gz, Ax, Ay, Az, Mx, My, Mz]
    # file1.writelines(
    #     ','.join(str(j) for j in imu_reading) + '\n')

    # print(Roll, Pitch, Yaw, Gx, Gy, Gz, Ax, Ay, Az, Mx, My, Mz)

    return imu_reading