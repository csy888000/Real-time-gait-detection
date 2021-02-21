# -*- coding: utf-8 -*-

import csv
import numpy as np
import pandas
from numpy import array
from numpy import mean
from numpy import std
from numpy import dstack
from pandas import read_csv
from tensorflow import keras
from keras.models import Sequential
from keras.layers import Dense
from keras.layers import Flatten
from keras.layers import Dropout
from keras.layers import LSTM
from keras.utils import to_categorical
from keras.models import load_model
from keras import backend
from matplotlib import pyplot
import pandas as pd
from sklearn import datasets, linear_model
from sklearn.model_selection import train_test_split
from matplotlib import pyplot as plt
import pandas
from numpy import array


def load_pretrained_model():
    pretrained_model1 = load_model('TrainedModel/IMUtoLatentFlat.h5')
    pretrained_model2 = load_model('TrainedModel/IMUtoLatentUp.h5')
    pretrained_model3 = load_model('TrainedModel/IMUtoLatentAcross.h5')
    pretrained_model4 = load_model('TrainedModel/IMUtoLatentDown.h5')
    activity_model_list = [pretrained_model1, pretrained_model2, pretrained_model3, pretrained_model4]
    return activity_model_list


def load_reference_angle():
    angle_filename_list = ['TrainedModel/jointAngleMeanFlat.txt', 'TrainedModel/jointAngleMeanUp.txt',
                      'TrainedModel/jointAngleMeanAcross.txt', 'TrainedModel/jointAngleMeanDown.txt']
    angle_ref_list = []
    for i in range(len(angle_filename_list)):
        angle_ref = pd.read_csv(angle_filename_list[i], sep=",", header=None)
        angle_ref = angle_ref.to_numpy()
        angle_ref_list.append(angle_ref)
    return angle_ref_list


# def get_phase_angle(angle_ref_list, type_walk):
#     angle_ref = angle_ref_list[type_walk-1]
#     angle_phase = angle_ref[:, 0]
#     angle_mean = angle_ref[:, 1:7]
#     return angle_phase, angle_mean


def load_mean_std():
    mean_filename_list = ['TrainedModel/meanFlat.txt', 'TrainedModel/meanUp.txt',
                 'TrainedModel/meanAcross.txt', 'TrainedModel/meanDown.txt']
    mean_list = []
    std_list = []
    for i in range(len(mean_filename_list)):
        mean_and_std = pd.read_csv(mean_filename_list[i], sep=",", header=None)
        mean_and_std = mean_and_std.to_numpy()
        xmean = mean_and_std[0, :]
        xstd = mean_and_std[1, :]
        mean_list.append(xmean)
        std_list.append(xstd)
    return mean_list, std_list


# def get_mean_std(mean_list, std_list, type_walk):
#     xmean = mean_list[type_walk-1]
#     xstd = std_list[type_walk - 1]
#     return xmean, xstd








