#!/usr/bin/env python

from IMU_receiver import *
from detection_algorithm import *
import numpy as np
from vicon_initialization import *
from vicon_receiver import *

from sklearn.preprocessing import StandardScaler
from sklearn.preprocessing import MinMaxScaler
import tensorflow as tf
physical_devices = tf.config.list_physical_devices('GPU')
tf.config.experimental.set_memory_growth(physical_devices[0], True)

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


    # pretrained_model = load_model('TrainedModel/detectActivity.h5')
    pretrained_model = load_model("TrainedModel/Roofinglstm - Classification.hdf5")
    regression_model = load_model("TrainedModel/Regressionlstm.hdf5")

    activity_model_list = load_pretrained_model()
    angle_ref_list = load_reference_angle()
    mean_list, std_list = load_mean_std()

    mean_activity, std_activity = load_mean_std_activity()


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
    IMU_data_all = []
    IMU_data_raw_list = []
    accuracy_list = []

    file_regression = r"New Final Full Data_Regression.csv"
    names = ['ax', 'ay', 'az', 'gx', 'gy', 'gz', 'acx', 'acy', 'acz', 'mx', 'my', 'mz', 'ID']
    # df = pandas.read_csv(file, names=names)
    df = pd.read_csv(file_regression, header=[0])
    # Split train data and test data
    train_size = int(len(df) * 0.8)
    train_dataset, test_dataset = df.iloc[:train_size], df.iloc[train_size:]
    # Split train data to X and y
    X_train = train_dataset.drop('ID', axis=1)
    y_train = train_dataset.loc[:, ['ID']]
    scaler_x = MinMaxScaler(feature_range=(0, 1))
    scaler_y = MinMaxScaler(feature_range=(0, 1))
    # Fit the scaler using available training data
    input_scaler = scaler_x.fit(X_train)
    output_scaler = scaler_y.fit(y_train)

    try:

        while True:
            time_start = time.time()
            IMU_data = child_conn_IMU.recv()
            IMU_data_raw_list.append(IMU_data)
            # IMU_data_all.append(IMU_data)
            IMU_data_list.insert(0,IMU_data)

            if len(IMU_data_list) > 1:
                IMU_data_diff = abs(np.array(IMU_data_list[0])-np.array(IMU_data_list[1]))
                for x,i in zip(IMU_data_diff[6:9], range(3)):
                    if x > 40:
                        IMU_data_list[0][i+6] = IMU_data_list[1][i+6]
                for x,i in zip(IMU_data_diff[3:6], range(3)):
                    if x > 500:
                        IMU_data_list[0][i+3] = IMU_data_list[1][i+3]
                IMU_data_all.append(IMU_data_list[0])


            if len(IMU_data_list) > 100:
                IMU_data_list = IMU_data_list[:-1]
                # IMU_data_list = IMU_data_list[1:]

            if vicon_enable:
                vicon_data = child_conn_vicon.recv()
                print("VICON data:", vicon_data[0])

            if current_frame > 100:
                scaler = StandardScaler()
                IMU_data_list_scaled = scaler.fit_transform(IMU_data_list)
                # imu_mean = np.mean(IMU_data_list, axis=0)
                # x2 = IMU_data_list - imu_mean
                # imu_std = np.std(x2, axis=0)  # normalize
                # x3 = x2/imu_std

                # inputstopoints_activity = IMU_data_list - mean_activity
                # inputstopoints_activity /= std_activity

                inputstopoints_test = np.expand_dims(IMU_data_list_scaled, axis=0)

                output_test_activity = test_model_activity(inputstopoints_test, pretrained_model)
                # output_test_activity = pretrained_model.predict(inputstopoints_test)
                # print(output_test_activity)
                output = np.amax(output_test_activity)
                output_idx = np.argmax(output_test_activity) + 1


                test_x_norm = input_scaler.transform(np.asarray(IMU_data_list[0:60]))
                regression_input = np.expand_dims(test_x_norm, axis=0)

                output_slope_angle = test_model_regression(regression_input, regression_model)
                output_slope_angle_scaled = scaler_y.inverse_transform(output_slope_angle[0])
                print(output_slope_angle_scaled)

                type_walk = output_idx
                xmean, xstd = get_mean_std(mean_list, std_list, type_walk)
                x = inputstopoints_test[:, 0:20, :] - xmean
                x /= xstd

                output_test = test_model(x, activity_model_list[type_walk-1])
                outputAll.append(output_test[0][0])
                current_propagation = output_test[0][0][3]

                angle_phase, angle_mean = get_phase_angle(angle_ref_list, type_walk)
                angle_interp = calculate_angle(current_propagation, angle_phase, angle_mean)

                angle_pred_list.append(angle_interp)
                propagation_list.append(current_propagation)
                output_idx_list.append(output_idx)
                accuracy_list.append(output_idx==3)
                accuracy = sum(accuracy_list)/len(accuracy_list)


                current_period = (time.time() - time_start) * 1000
                period_list.append(current_period)
                # print(sum(period_list) / len(period_list))

                print("        %6d,       %10.4f      %6d,       %10.4f,     %10.4f" % (
                    current_frame, sum(period_list) / len(period_list), output_idx, current_propagation, accuracy))

                data_record = [current_frame, current_period, IMU_data, output_idx, current_propagation]
                data_record_list.append(data_record)

                file1.writelines(','.join(str(j) for j in data_record) + '\n')

            current_frame += 1

    except KeyboardInterrupt:

        serial_connection.close()
        file1.close()
        print('Program Interrupted! Closing...')

    plt.plot(output_idx_list)
    plt.title('Activity')
    plt.ylabel('Activity')
    plt.xlabel('Frame')
    plt.legend(['activity'], loc='upper left')
    plt.show()

    IMU_data_all = np.asarray(IMU_data_all)
    plt.plot(IMU_data_all[:,8])
    plt.title('IMU')
    plt.ylabel('IMU')
    plt.xlabel('Frame')
    plt.legend(['IMU'], loc='upper left')
    plt.show()

    IMU_data_raw_list = np.asarray(IMU_data_raw_list)
    plt.plot(IMU_data_raw_list[:,8])
    plt.title('IMU raw')
    plt.ylabel('IMU raw')
    plt.xlabel('Frame')
    plt.legend(['IMU raw'], loc='upper left')
    plt.show()





