import sys
sys.path.append('/usr/local/lib/python3.8/dist-packages')

from pyvicon import *
from vicon_initialization import *
from vicon_receiver import *
import time


if __name__ == '__main__':
    test = PyVicon()
    print("SDK version : {}".format(test.__version__))
    test.connect("192.168.10.203")
    print("Connection status : {}".format(test.is_connected()))

    test.set_stream_mode(StreamMode.ClientPull)


    test.enable_marker_data()
    test.enable_segment_data()
    try:
        while True:
            start = time.time()

            test.get_frame()
            f_num = test.get_frame_number()
            print("Frame number is", f_num)
            if test.get_subject_count():
                subject_name = test.get_subject_name(0)
                marker_count = test.get_marker_count(subject_name)
                print("The", subject_name, "has", marker_count, "markers")
                for i in range(marker_count):
                    marker_name = test.get_marker_name(subject_name, i)
                    print("The marker", marker_name, "is located at:", test.get_marker_global_translation(subject_name, marker_name))

            now = time.time()

            print("It takes:", now-start, "to run this")

    except KeyboardInterrupt:

        test.disconnect()
        print('Program Interrupted! Closing...')

