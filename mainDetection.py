#!/usr/bin/env python

from IMU_receiver import *
from detection_algorithm import *
import numpy as np
from vicon_initialization import *
from vicon_receiver import *


if __name__ == '__main__':
    serial_connection = IMU_init()

    (parent_conn_IMU, child_conn_IMU) = Pipe()
    sender_IMU = Process(target = async_IMU, args = (parent_conn_IMU, serial_connection))
    sender_IMU.start()

    # vicon_stream, subject_name, marker_count = vicon_init("192.168.10.203")
    IP_ADDRESS = "192.168.10.203"
    vicon_enable = False
    if vicon_enable:
        (parent_conn_vicon, child_conn_vicon) = Pipe()
        sender_vicon = Process(target = async_vicon, args = (parent_conn_vicon, IP_ADDRESS))
        sender_vicon.start()


    pretrained_model = load_model('TrainedModel/detectActivity.h5')
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
    DATA_LEN = 10
    propagation_list = []
    output_idx_list = []


    try:

        while True:
            time_start = time.time()
            IMU_data = child_conn_IMU.recv()
            IMU_data_list.append(IMU_data)
            if len(IMU_data_list) > 10:
                IMU_data_list = IMU_data_list[1:]

            if vicon_enable:
                vicon_data = child_conn_vicon.recv()
                print("VICON data:", vicon_data[0])

            if current_frame > 10:
                inputstopoints_test = np.expand_dims(IMU_data_list, axis=0)
                output_test = test_model_activity(inputstopoints_test, pretrained_model)
                output = np.amax(output_test)
                output_idx = np.argmax(output_test) + 1


                type_walk = output_idx
                xmean, xstd = get_mean_std(mean_list, std_list, type_walk)
                x = inputstopoints_test - xmean
                x /= xstd

                output_test = test_model(x, activity_model_list[type_walk-1])
                outputAll.append(output_test[0][0])
                current_propagation = output_test[0][0][3]

                angle_phase, angle_mean = get_phase_angle(angle_ref_list, type_walk)
                angle_interp = calculate_angle(current_propagation, angle_phase, angle_mean)

                angle_pred_list.append(angle_interp)
                propagation_list.append(current_propagation)
                output_idx_list.append(output_idx)

                current_period = (time.time() - time_start) * 1000
                period_list.append(current_period)
                # print(sum(period_list) / len(period_list))

                print("        %6d,       %10.4f      %6d,       %10.4f" % (
                    current_frame, sum(period_list) / len(period_list), output_idx, current_propagation))

                imu_reading = [current_frame, current_period, output_idx, current_propagation, IMU_data]
                file1.writelines(','.join(str(j) for j in imu_reading) + '\n')

            current_frame += 1

    except KeyboardInterrupt:

        serial_connection.close()
        file1.close()
        print('Program Interrupted! Closing...')



    # print(np.asarray(angle_pred_list))
    # np.savetxt('TrainedModel/anglePred.txt', np.asarray(angle_pred_list), delimiter=',')
    # np.savetxt('TrainedModel/outputAll.txt', np.asarray(outputAll), delimiter=',')
