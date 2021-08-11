from IMU_initialization import *
from IMU_send import *

serial_conn = IMU_init()
current_frame = 0
while True:
    imu_reading = receive_from_IMU(serial_conn)
    # print(imu_reading)
    current_frame += 1

