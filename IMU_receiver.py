from multiprocessing import Process, Pipe
from IMU_initialization import *
from IMU_send import *


def async_IMU(pipe, serial_conn):
    num = 0

    while True:
        imu_reading = receive_from_IMU(serial_conn)
        pipe.send(imu_reading)
        # print(num)
        num += 1
    pipe.close()