# using Vicon datastream sdk to stream vicon data live
import time
import struct
from pyvicon import *


def receive_from_vicon(vicon_stream, subject_name, marker_count):
    vicon_reading = [0 for i in range(marker_count)]
    # vicon_reading = []

    # try:
    #     start = time.time()
        # print("Loop here")
        # frame_result = vicon_stream.get_frame()
        # f_num = vicon_stream.get_frame_number()
        # print(f_num)
        # print(frame_result)

        # time_now = time.time() - start
        # print(time_now)

    if vicon_stream.get_subject_count():
        # subject_name = vicon_stream.get_subject_name(0)
        # marker_count = vicon_stream.get_marker_count(subject_name)
        # print("The", subject_name, "has", marker_count, "markers")
        # print("Subject count is", vicon_stream.get_subject_count())
        for i in range(marker_count):
            marker_name = vicon_stream.get_marker_name(subject_name, i)
            # print("The marker", marker_name, "is located at:",
            #       vicon_stream.get_marker_global_translation(subject_name, marker_name))
            vicon_reading[i] = vicon_stream.get_marker_global_translation(subject_name, marker_name)
            # current_reading = vicon_stream.get_marker_global_translation(subject_name, marker_name)
            # if current_reading == None:
            #     current_reading = [0,0,0]
            # vicon_reading.append(current_reading)
            # print("current read is", current_reading)
        # now = time.time()

        # print("It takes:", now - start, "to run this")

    # except KeyboardInterrupt:
    #
    #     vicon_stream.disconnect()
    #     print('Program Interrupted! Closing...')

    return vicon_reading
