import csv
import numpy as np
import pandas
from numpy import array
from numpy import mean
from numpy import std
from numpy import dstack
from pandas import read_csv
from tensorflow import keras
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense
from tensorflow.keras.layers import Flatten
from tensorflow.keras.layers import Dropout
from tensorflow.keras.layers import LSTM
from tensorflow.keras.utils import to_categorical
from tensorflow.keras.models import load_model
from tensorflow.keras import backend
from matplotlib import pyplot
import pandas as pd
from sklearn import datasets, linear_model
from sklearn.model_selection import train_test_split
from matplotlib import pyplot as plt
import pandas
from numpy import array
from detection_initialization import *


def get_phase_angle(angle_ref_list, type_walk):
    angle_ref = angle_ref_list[type_walk-1]
    angle_phase = angle_ref[:, 0]
    angle_mean = angle_ref[:, 1:7]
    return angle_phase, angle_mean


def get_mean_std(mean_list, std_list, type_walk):
    xmean = mean_list[type_walk-1]
    xstd = std_list[type_walk - 1]
    return xmean, xstd


def calculate_angle(tt, anglePro, angleMean):
    if tt > 1:
        tt = 1
    elif tt < 0:
        tt = 0
    stateG = np.where(anglePro >= tt)
    tstate = min(stateG[0])

    angleInterp = []
    for j in range(6):
        t1 = anglePro[tstate - 1]
        t2 = anglePro[tstate]

        angle1 = angleMean[tstate - 1, j]
        angle2 = angleMean[tstate, j]
        anglet = angle1 + (angle2 - angle1) * (tt - t1) / (t2 - t1)
        angleInterp.append(anglet)
    return angleInterp


def test_size(size, inputdata, testlabel):
    tx = []
    ty = []

    i = 0
    while (i + size) <= len(inputdata) - 1:
        tx.append(inputdata[i: i + size])
        ty.append(testlabel[i + size])

        i += 1
    assert len(tx) == len(ty)
    return (tx, ty)


def test_model(inputstopoints, pretrained_model):
    get_dense_output = backend.function([pretrained_model.layers[0].input], [pretrained_model.layers[4].output])
    dense_17layer_output = get_dense_output(inputstopoints)
    dense_17layer_output = np.asarray(dense_17layer_output)
    return dense_17layer_output


def test_size_x(size, inputdata):
    tx = []
    i = 0
    while (i + size) <= len(inputdata) - 1:
        tx.append(inputdata[i: i + size])
        i += 1
    return tx


def test_model_activity(inputstopoints, pretrained_model):
    get_dense_output_activity = backend.function([pretrained_model.layers[0].input], [pretrained_model.layers[3].output])
    dense_17layer_output = get_dense_output_activity(inputstopoints)
    dense_17layer_output = np.asarray(dense_17layer_output)
    return dense_17layer_output


def test_model_regression(inputstopoints, pretrained_model):
    get_dense_output_activity = backend.function([pretrained_model.layers[0].input], [pretrained_model.layers[4].output])
    dense_17layer_output = get_dense_output_activity(inputstopoints)
    dense_17layer_output = np.asarray(dense_17layer_output)
    return dense_17layer_output



if __name__ == '__main__':
    pretrained_model = load_model('TrainedModel/detectActivity.h5')
    activity_model_list = load_pretrained_model()
    angle_ref_list = load_reference_angle()
    mean_list, std_list = load_mean_std()


    # FILENAME_TEST="IMU_walk_20deg_clockwise.csv"
    FILENAME_TEST = "TrainedModel/Book1.csv"
    testdata = pandas.read_csv(FILENAME_TEST)
    testdata = testdata.to_numpy()
    labeldata = testdata[:, 12]
    testAngle = testdata[:, 13:19]
    testdata = testdata[:, 0:12]

    DATA_LEN = 10
    tx = test_size_x(DATA_LEN, testdata)
    print("tx shape: {}".format(np.array(tx).shape))



    j = 0
    totalResult = []
    error_sum = 0
    ty_sum = 0
    angle_error_sum = 0
    angle_pred_list = []
    outputAll = []
    while j < len(tx):
        inputstopoints_test = np.expand_dims(tx[j], axis=0)
        print(inputstopoints_test)
        output_test = test_model_activity(inputstopoints_test, pretrained_model)
        output = np.amax(output_test)
        output_idx = np.argmax(output_test) + 1
        label = labeldata[j]
        totalResult.append(label == output_idx)
        accuracy = np.sum(totalResult) / len(totalResult) * 100


        type_walk = output_idx
        xmean, xstd = get_mean_std(mean_list, std_list, type_walk)
        x = inputstopoints_test - xmean
        x /= xstd

        output_test = test_model(x, activity_model_list[type_walk-1])
        outputAll.append(output_test[0][0])
        currentPropagation = output_test[0][0][3]

        angle_phase, angle_mean = get_phase_angle(angle_ref_list, type_walk)
        angle_interp = calculate_angle(currentPropagation, angle_phase, angle_mean)

        angle_pred_list.append(angle_interp)
        angle_error = sum(abs(angle_interp - testAngle[j])) / 6
        angle_error_sum += angle_error
        angle_error_mean = angle_error_sum / j
        print("        %6d,      %6d,      %6d,   %10.5f,       %10.4f,         %10.4f,       %10.4f" % (
            j, label, output_idx, accuracy, currentPropagation, angle_error, angle_error_mean))

        j = j + 1

    # Current Frame, Real Label,   Pred Label,    Accuracy,     Current Phase,      Current Error,      Total Error



    print(np.asarray(angle_pred_list))
    np.savetxt('TrainedModel/anglePred.txt', np.asarray(angle_pred_list), delimiter=',')
    np.savetxt('TrainedModel/outputAll.txt', np.asarray(outputAll), delimiter=',')


