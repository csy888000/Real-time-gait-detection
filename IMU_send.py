import serial
import time
import struct
import numpy as np

g = 9.81
convL = 16.0 / 32768.0 * g  # m/s**2
convAv = 2000.0 / 32768.0  # deg/s
convA = 180.0 / 32768  # deg
convM = 1.0  #
rad2deg = 180 / np.pi
time_list = []
# num = 0

def receive_from_IMU(serial_conn):
    try:
        BlueIMU = True
        RaspIMU = True

        if not BlueIMU:
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
        elif not RaspIMU:
            time_start = time.time()

            bytes = serial_conn.read(39)  # Each package has 39 bytes

            start_index = bytes.find(b'\x3A\x01\x00')
            end_index = bytes.find(b'\x0D\x0A', start_index + 1)

            time_index = start_index + 7
            mag_index = start_index + 11
            ang_vel_index = start_index + 17
            angle_index = start_index + 23
            acc_index = start_index + 29

            time_bytes = bytes[time_index:time_index + 4]
            time_imu = struct.unpack("<I", time_bytes)[0] / 400

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
            imu_reading = [ang_x, ang_y, ang_z, ang_vel_x, ang_vel_y, ang_vel_z, acc_x, acc_y, acc_z, mag_x,
                           mag_y, mag_z]
        else:
            time_start = time.time()
            # Header: 3 bytes; Each IMU has 56 bytes; Up to seven IMUs
            # Each IMU: time * 1, angle_vel * 3, angle * 3, lin acc * 3, quaternion * 4
            # All data in float
            bytes = serial_conn.read(399)
            # if time.time() - time_start > 0.0:
                # print(bytes)

            start_index = bytes.find(b'\x3A\x88')
            imu_reading = []
            if start_index != 0:
                bytes_ignore = serial_conn.read(start_index)
                # print(bytes_ignore)
            else:
                end_index = bytes.find(b'\x0D\x0A', start_index + 1)
                # print(start_index, end_index)

                # Up to seven IMUs
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



    except KeyboardInterrupt:

        serial_conn.close()
        print('Program Interrupted! Closing...')

    return imu_reading
