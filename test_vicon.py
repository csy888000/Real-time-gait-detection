import sys
sys.path.append('/usr/local/lib/python3.8/dist-packages')

from pyvicon import *
from vicon_initialization import *
from vicon_receiver import *
import time


if __name__ == '__main__':

    IP_ADDRESS = "192.168.10.203"
    # vicon_stream = vicon_init("192.168.10.203")
    # print("Vicon connection is", vicon_stream)
    # print("Subject name is", subject_name)
    # print("Marker counts is", marker_count)

    # subject_name = "ComTest"
    # marker_count = 3
    (parent_conn_vicon, child_conn_vicon) = Pipe()
    sender_vicon = Process(target = async_vicon, args = (parent_conn_vicon, IP_ADDRESS))
    sender_vicon.start()
    # sender_vicon.join()

    try:

        while True:
            time_start = time.time()

            # time_now = time.time() - time_start
            # print(time_now)
            vicon_data = child_conn_vicon.recv()
            time_period = time.time() - time_start
            print("VICON data:", vicon_data)
            print("Vicon period is", time_period)

    except KeyboardInterrupt:

        vicon_stream.disconnect()
        print('Program Interrupted! Closing...')



