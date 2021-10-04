#!/usr/bin/env python

from IMU_receiver import *
from detection_algorithm import *
import numpy as np
from vicon_initialization import *
from vicon_receiver import *

import tensorflow as tf
physical_devices = tf.config.list_physical_devices('GPU')
tf.config.experimental.set_memory_growth(physical_devices[0], True)

if __name__ == '__main__':
    serial_connection = IMU_init()

    (parent_conn_IMU, child_conn_IMU) = Pipe()
    sender_IMU = Process(target = async_IMU, args = (parent_conn_IMU, serial_connection))
    sender_IMU.start()

    #########################################################################################
    # vicon_stream, subject_name, marker_count = vicon_init("192.168.10.203")
    IP_ADDRESS = "192.168.1.10"
    vicon_enable = False
    #########################################################################################


    if vicon_enable:
        (parent_conn_vicon, child_conn_vicon) = Pipe()
        sender_vicon = Process(target = async_vicon, args = (parent_conn_vicon, IP_ADDRESS))
        sender_vicon.start()


    # pretrained_model = load_model('TrainedModel/detectActivity.h5')
    pretrained_model = load_model("TrainedModel/Roofinglstm - Classification.hdf5")
    activity_model_list = load_pretrained_model()
    angle_ref_list = load_reference_angle()
    mean_list, std_list = load_mean_std()

    timestr = time.strftime("%Y%m%d-%H%M%S")
    file_name = "Data/" + "data-" + timestr + ".txt"
    file1 = open(file_name, "w")


    period_list = []
    current_frame = 0
    IMU_data_list = []
    current_f = 0
    totalResult = []
    error_sum = 0
    ty_sum = 0
    angle_error_sum = 0
    angle_pred_list = []
    outputAll = []
    DATA_LEN = 20
    propagation_list = []
    output_idx_list = []
    data_record_list = []

    try:

        while True:
            time_start = time.time()
            # IMUs: Trunk, left thigh, left shank, right thigh, right shank
            # Data: Periods, angles, angle velocities, quaternion
            # Range of measurement: x, y, z as follows; x+ facing left, y+ facing up, z+ facing forward
            # https://lp-research.atlassian.net/wiki/spaces/LKB/pages/1100611599/LPMS+User+Manual
            IMU_data = child_conn_IMU.recv()

            if vicon_enable:
                vicon_data = child_conn_vicon.recv()
                print("VICON data:", vicon_data[0])

            current_period = (time.time() - time_start) * 1000

            # Data format of output is plain txt
            data_record = [current_frame, current_period]
            if vicon_enable:
                data_record.extend(vicon_data)
                data_record.extend(IMU_data)
            else:
                data_record.extend(IMU_data)
                if IMU_data:
                    print("%10.4f,      %10.4f,      %10.4f" % (IMU_data[43], IMU_data[44], IMU_data[45]))
            data_record_list.append(data_record)

            file1.writelines(','.join(str(j) for j in data_record) + '\n')

            current_frame += 1

    except KeyboardInterrupt:

        serial_connection.close()
        file1.close()
        print('Program Interrupted! Closing...')








