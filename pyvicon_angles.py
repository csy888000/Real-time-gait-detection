import pyvicon_module

from enum import Enum
import numpy as np
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation


class Marker(Enum):
    LFHD = 0
    RFHD = 1
    LBHD = 2
    RBHD = 3
    C7 = 4
    T10 = 5
    CLAV = 6
    STRN = 7
    RBAK = 8
    LSHO = 9
    LUPA = 10
    LELB = 11
    LFRM = 12
    LWRA = 13
    LWRB = 14
    LFIN = 15
    RSHO = 16
    RUPA = 17
    RELB = 18
    RFRM = 19
    RWRA = 20
    RWRB = 21
    RFIN = 22
    LASI = 23
    RASI = 24
    LPSI = 25
    RPSI = 26
    LTHI = 27
    LKNE = 28
    LTIB = 29
    LANK = 30
    LHEE = 31
    LTOE = 32
    RTHI = 33
    RKNE = 34
    RTIB = 35
    RANK = 36
    RHEE = 37
    RTOE = 38

def RotZ(angle):
    return np.array([[np.cos(angle), -np.sin(angle), 0],
                     [np.sin(angle),  np.cos(angle), 0],
                     [    0,                0,       1]])

class Result(Enum):
    Unknown = 0
    NotImplemented = 1
    Success = 2
    InvalidHostName = 3
    InvalidMulticastIP = 4
    ClientAlreadyConnected = 5
    ClientConnectionFailed = 6
    ServerAlreadyTransmittingMulticast = 7
    ServerNotTransmittingMulticast = 8
    NotConnected = 9
    NoFrame = 10
    InvalidIndex = 11
    InvalidCameraName = 12
    InvalidSubjectName = 13
    InvalidSegmentName = 14
    InvalidMarkerName = 15
    InvalidDeviceName = 16
    InvalidDeviceOutputName = 17
    InvalidLatencySampleName = 18
    CoLinearAxes = 19
    LeftHandedAxes = 20
    HapticAlreadySet = 21


class StreamMode(Enum):
    ClientPull = 0
    ClientPullPreFetch = 1
    ServerPush = 2


class Direction(Enum):
    Up = 0
    Down = 1
    Left = 2
    Right = 3
    Forward = 4
    Backward = 5


class TimecodeStandard(Enum):
    NONE = 0
    PAL = 1
    NTSC = 2
    NTSCDrop= 3
    Film = 4


class PyVicon:
    def __init__(self):
        self.client_ = pyvicon_module.new_client()
        major, minor, point = pyvicon_module.pyvicon_version(self.client_)
        self.__version__ = "{}.{}.{}".format(major, minor, point)

    def connect(self, ip):
        return Result(pyvicon_module.pyvicon_connect(self.client_, ip))

    def connect_multicast(self, ip, multicast):
        return Result(pyvicon_module.pyvicon_connect_to_multicast(self.client_, ip, multicast))

    def disconnect(self):
        return Result(pyvicon_module.pyvicon_disconnect(self.client_))

    def is_connected(self):
        return pyvicon_module.pyvicon_isconnected(self.client_)

    def start_server_multicast(self, server_ip, multicast_ip):
        return Result(pyvicon_module.pyvicon_start_transmitting_multicast(self.client_, server_ip, multicast_ip))

    def stop_server_multicast(self):
        return Result(pyvicon_module.pyvicon_stop_transmitting_multicast(self.client_))

    def enable_segment_data(self):
        return Result(pyvicon_module.pyvicon_enable_segment_data(self.client_))

    def enable_marker_data(self):
        return Result(pyvicon_module.pyvicon_enable_marker_data(self.client_))

    def enable_unlabeled_marker_data(self):
        return Result(pyvicon_module.pyvicon_enable_unlabeled_marker_data(self.client_))

    def enable_device_data(self):
        return Result(pyvicon_module.pyvicon_enable_device_data(self.client_))

    def disable_segment_data(self):
        return Result(pyvicon_module.pyvicon_disable_segment_data(self.client_))

    def disable_marker_data(self):
        return Result(pyvicon_module.pyvicon_disable_marker_data(self.client_))

    def disable_unlabeled_marker_data(self):
        return Result(pyvicon_module.pyvicon_disable_unlabeled_marker_data(self.client_))

    def disable_device_data(self):
        return Result(pyvicon_module.pyvicon_disable_device_data(self.client_))

    def is_marker_data_enabled(self):
        return pyvicon_module.pyvicon_is_marker_data_enabled(self.client_)

    def is_unlabeled_marker_data_enabled(self):
        return pyvicon_module.pyvicon_is_unlabeled_marker_data_enabled(self.client_)

    def is_device_data_enabled(self):
        return pyvicon_module.pyvicon_is_device_data_enabled(self.client_)

    def set_buffer_size(self, size):
        pyvicon_module.pyvicon_set_buffer_size(self.client_, size)

    def set_stream_mode(self, stream_mode):
        self.check_enum(stream_mode, StreamMode, "set_stream_mode")
        return Result(pyvicon_module.pyvicon_set_stream_mode(self.client_, stream_mode.value))

    def set_axis_mapping(self, X, Y, Z):
        """
        Common : Z-up (Forward, Left, Up)
                 Y-up (Forward, Up, Right)
        """
        self.check_enum(X, Direction, "set_axis_mapping")
        self.check_enum(Y, Direction, "set_axis_mapping")
        self.check_enum(Z, Direction, "set_axis_mapping")
        return Result(pyvicon_module.pyvicon_set_axis_mapping(self.client_, X.value, Y.value, Z.value))

    def get_axis_mapping(self):
        X, Y, Z = pyvicon_module.pyvicon_get_axis_mapping(self.client_)
        return Direction(X), Direction(Y), Direction(Z)

    def get_frame(self):
        return Result(pyvicon_module.pyvicon_get_frame(self.client_))

    def get_frame_number(self):
        return pyvicon_module.pyvicon_get_frame_number(self.client_)

    def get_time_code(self):
        result, standard, hours, minutes, seconds = pyvicon_module.pyvicon_get_time_code(self.client_)
        return Result(result), TimecodeStandard(standard), hours, minutes, seconds

    def get_frame_rate(self):
        return pyvicon_module.pyvicon_get_frame_rate(self.client_)

    def get_latency_total(self):
        return pyvicon_module.pyvicon_get_latency_total(self.client_)

    def get_subject_count(self):
        return pyvicon_module.pyvicon_get_subject_count(self.client_)

    def get_subject_name(self, index):
        return pyvicon_module.pyvicon_get_subject_name(self.client_, index)

    def get_subject_root_segment_name(self, name):
        return pyvicon_module.pyvicon_get_subject_root_segment_name(self.client_, name)

    def get_segment_global_translation(self, subject_name, segment_name):
        T = pyvicon_module.pyvicon_get_segment_global_translation(self.client_, subject_name, segment_name)
        if T is None:
            return None
        return np.array(T)

    def get_segment_global_rotation_matrix(self, subject_name, segment_name):
        matrix = pyvicon_module.pyvicon_get_segment_global_rotation_matrix(self.client_, subject_name, segment_name)
        if matrix is None:
            return None
        return np.array(matrix).reshape((3, 3))

    def get_segment_global_quaternion(self, subject_name, segment_name):
        """
        Return the quaternion as : w,x,y,z
        :param subject_name:
        :param segment_name:
        :return:
        """
        quaternion = pyvicon_module.pyvicon_get_segment_global_quaternion(self.client_, subject_name, segment_name)
        if quaternion is None:
            return None
        return np.array(quaternion)

    def get_subject_quality(self, name):
        return pyvicon_module.pyvicon_get_object_quality(self.client_, name)

    def get_marker_count(self, name):
        return pyvicon_module.pyvicon_get_marker_count(self.client_, name)

    def get_marker_name(self, name, index):
        return pyvicon_module.pyvicon_get_marker_name(self.client_, name, index)

    def get_marker_global_translation(self, subject_name, marker_name):
        pose = pyvicon_module.pyvicon_get_marker_global_translation(self.client_, subject_name, marker_name)
        if pose is None:
            return None
        return np.array(pose)

    def get_unlabeled_marker_count(self):
        return pyvicon_module.pyvicon_get_unlabeled_marker_count(self.client_)

    def get_unlabeled_marker_global_translation(self, index):
        return pyvicon_module.pyvicon_get_unlabeled_marker_global_translation(self.client_, index)

    def get_camera_count(self):
        return pyvicon_module.pyvicon_get_camera_count(self.client_)

    def get_camera_name(self, index):
        return pyvicon_module.pyvicon_get_camera_name(self.client_, index)

    @staticmethod
    def check_enum(value, enum, method):
        if not isinstance(value, enum):
            raise RuntimeError("{} expect a {}, received a {}".format(method, enum, type(value)))

if __name__ == '__main__':
    test = PyVicon()
    print("SDK version : {}".format(test.__version__))
        
    test.connect("192.168.10.203")
    print("Connection is :", test.is_connected())
    
    test.set_stream_mode(StreamMode.ClientPull)

    test.enable_marker_data()
    
    test.enable_segment_data()

    while True:
        start = time.time()
        test.get_frame()
        # print(test.get_marker_global_translation("ComTest","ComTest1"))
        subject_name = test.get_subject_name(0)
        # for i in range(test.get_marker_count(subject_name)):
        #
        LASI = test.get_marker_global_translation(subject_name, "LASI")
        LPSI = test.get_marker_global_translation(subject_name, "LPSI")
        RASI = test.get_marker_global_translation(subject_name, "RASI")
        RPSI = test.get_marker_global_translation(subject_name, "RPSI")
        frontMid = 0.5 * (LASI + RASI)
        rearMid = 0.5 * (LPSI + RPSI)

        theta = np.arctan2(frontMid[1]-rearMid[1],frontMid[0]-rearMid[0])
        rotMat = RotZ(-theta)

        LSHO_r = rotMat @ test.get_marker_global_translation(subject_name, "LSHO")
        RSHO_r = rotMat @ test.get_marker_global_translation(subject_name, "RSHO")
        LELB_r = rotMat @ test.get_marker_global_translation(subject_name, "LELB")
        RELB_r = rotMat @ test.get_marker_global_translation(subject_name, "RELB")
        LWRA_r = rotMat @ test.get_marker_global_translation(subject_name, "LWRA")
        RWRA_r = rotMat @ test.get_marker_global_translation(subject_name, "RWRA")
        LWRB_r = rotMat @ test.get_marker_global_translation(subject_name, "LWRB")
        RWRB_r = rotMat @ test.get_marker_global_translation(subject_name, "RWRB")
        LFIN_r = rotMat @ test.get_marker_global_translation(subject_name, "LFIN")
        RFIN_r = rotMat @ test.get_marker_global_translation(subject_name, "RFIN")
        LWRM_r = 0.5 * (LWRA_r + LWRB_r)
        RWRM_r = 0.5 * (RWRA_r + RWRB_r)
        LASI_r = rotMat @ LASI
        RASI_r = rotMat @ RASI
        LPSI_r = rotMat @ LPSI
        RPSI_r = rotMat @ RPSI
        LHIP_r = 0.5 * (LASI_r + LPSI_r)
        RHIP_r = 0.5 * (RASI_r + RPSI_r)
        LKNE_r = rotMat @ test.get_marker_global_translation(subject_name, "LKNE")
        RKNE_r = rotMat @ test.get_marker_global_translation(subject_name, "RKNE")
        LANK_r = rotMat @ test.get_marker_global_translation(subject_name, "LANK")
        RANK_r = rotMat @ test.get_marker_global_translation(subject_name, "RANK")
        LHEE_r = rotMat @ test.get_marker_global_translation(subject_name, "LHEE")
        RHEE_r = rotMat @ test.get_marker_global_translation(subject_name, "RHEE")
        LTOE_r = rotMat @ test.get_marker_global_translation(subject_name, "LTOE")
        RTOE_r = rotMat @ test.get_marker_global_translation(subject_name, "RTOE")

        thetaLeftHand = np.arctan2(LWRM_r[2]-LFIN_r[2], LWRM_r[0]-LFIN_r[0]) * 180 / np.pi - 90
        thetaRightHand = np.arctan2(RWRM_r[2]-RFIN_r[2], RWRM_r[0]-RFIN_r[0]) * 180 / np.pi - 90
        thetaLeftForeArm = np.arctan2(LELB_r[2]-LWRM_r[2], LELB_r[0]-LWRM_r[0]) * 180 / np.pi - 90
        thetaRightForeArm = np.arctan2(RELB_r[2]-RWRM_r[2], RELB_r[0]-RWRM_r[0]) * 180 / np.pi - 90
        thetaLeftShoulder = np.arctan2(LSHO_r[2]-LELB_r[2], LSHO_r[0]-LELB_r[0]) * 180 / np.pi - 90
        thetaRightShoulder = np.arctan2(RSHO_r[2]-RELB_r[2], RSHO_r[0]-RELB_r[0]) * 180 / np.pi - 90

        thetaLeftFoot = np.arctan2(LTOE_r[2]-LHEE_r[2], LTOE_r[0]-LHEE_r[0]) * 180 / np.pi
        thetaRightFoot = np.arctan2(RTOE_r[2]-RHEE_r[2], RTOE_r[0]-RHEE_r[0]) * 180 / np.pi
        thetaLeftCalf = np.arctan2(LKNE_r[2]-LANK_r[2], LKNE_r[0]-LANK_r[0]) * 180 / np.pi - 90
        thetaRightCalf = np.arctan2(RKNE_r[2]-RANK_r[2], RKNE_r[0]-RANK_r[0]) * 180 / np.pi - 90
        thetaLeftHip = np.arctan2(LHIP_r[2]-LKNE_r[2], LHIP_r[0]-LKNE_r[0]) * 180 / np.pi - 90
        thetaRightHip = np.arctan2(RHIP_r[2]-RKNE_r[2], RHIP_r[0]-RKNE_r[0]) * 180 / np.pi - 90

        thetaLeftElbow = thetaLeftForeArm - thetaLeftShoulder
        thetaRightElbow = thetaRightForeArm - thetaRightShoulder
        thetaLeftWrist = thetaLeftHand - thetaLeftForeArm
        thetaRightWrist = thetaRightHand - thetaRightForeArm
        thetaLeftKnee = thetaLeftCalf - thetaLeftHip
        thetaRightKnee = thetaRightCalf - thetaRightHip
        thetaLeftAnkle = thetaLeftFoot - thetaLeftCalf
        thetaRightAnkle = thetaRightFoot - thetaRightCalf

        jointAngles = [thetaLeftHip, thetaRightHip, thetaLeftKnee, thetaRightKnee,
                       thetaLeftAnkle, thetaRightAnkle, theta, thetaLeftShoulder,
                       thetaRightShoulder, thetaLeftElbow, thetaRightElbow, thetaLeftWrist,
                       thetaRightWrist]

        print(jointAngles)


        now = time.time()

