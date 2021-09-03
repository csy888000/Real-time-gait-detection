import serial
import time


def IMU_init():
    g = 9.81
    convL = 16.0 / 32768.0 * g  # m/s**2
    convAv = 2000.0 / 32768.0  # deg/s
    convA = 180.0 / 32768  # deg
    convM = 1.0  #

    # serialPort = '/dev/rfcomm0'  # /dev/ttyUSB0 # change this line if bluetooth or other port
    # serialPort = '/dev/ttyUSB0'
    serialPort = '/dev/ttyACM0'
    serialBaud = 115200

    # start = time.time()
    # timestr = time.strftime("%Y%m%d-%H%M%S")
    # file_name = "Data/" + "data-" + timestr + ".txt"
    # file1 = open(file_name, "w")
    # file1.write("Roll,Pitch,Yaw,Gx,Gy,Gz, Ax,Ay,Az,,Mx,My,Mz,\n")


    # Connect to serial port
    print('Trying to connect to ' + str(serialPort) +
          ' at ' + str(serialBaud) + ' BAUD.')
    try:
        serial_conn = serial.Serial(serialPort, serialBaud, timeout=None)
        print('Connected!')
    except:
        print("Failed to connect with " + str(serialPort) +
              ' at ' + str(serialBaud) + ' BAUD.')

    return serial_conn
