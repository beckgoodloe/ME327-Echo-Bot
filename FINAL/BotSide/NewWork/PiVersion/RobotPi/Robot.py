#
# File: Robot.py
# ------------------------------
# Authors: Erick Blankenberg, Beck Goodloe, Josiah Clark
# Date: 5/23/2018
#
# Description:
#   Recieves directional information and transmits lidar data
#   and bumper states to teleoperator client. Uses TCP
#

import rplidar
import msgpack
import socket
import serial
import signal
from serial.tools import list_ports
import numpy as np

def signal_term_handler(signal, frame):
    lidarUnit.stop()
    lidarUnit.stop_motor()
    exit()

# Connects to microcontroller and lidar
portList  = list_ports.comports()
MCUUnit   = None
lidarUnit = None
print("Seeking Peripherals")
MCUUnit = getMCUPort('Here!')
if not MCUUnit:
    print("Connection to MCU Failed")
else:
    print("Connected to MCU!")
"""
if not lidarUnit:
    print("Connection to Lidar Failed")
"""


# Connects to operator
OPERATOR_IP = '25.11.192.247'
PORT        = 65432
print("Seeking Host")
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((OPERATOR_IP, PORT))

print("Operational!")
while True: # Waits on teleoperator to send update
    # Gets most recent data packet
    data = s.recv(1024)

    # Updates robot movement (note that if hanging the MCU will stop moving after TIMEOUT seconds)
    if MCUUnit:
        MCUUnit.write(data) # TODO ACTUALLY DECODE THIS
    print("T>B %s:" % (data.decode()))

    # Broadcasts some lidar data
    """
    if lidarUnit:
        try:
            data = next(lidarUnit.iter_scans(min_len = 40))
            dataToSend = msgpack.packb(data)
            s.send(dataToSend)
        except:
            raise
            pass
    """

def getMCUPort(expectedResponse):
    print(portList)
    for currentPortIndex in range(0, len(portList)):
        currentPort = portList[currentPortIndex]
        # May be Lidar FIXME
        """
        try:
            lidarUnit = rplidar.RPLidar(currentPort[0])
            print("Connected to Lidar!")
            lidarUnit.stop()
            lidarUnit.stop_motor()
            lidarUnit.clean_input() # :(
            signal.signal(signal.SIGTERM, signal_term_handler)
            break
        except rplidar.RPLidarException as e:
            print(e)
            pass
        """
        # May be MCU
        try:
            serialPort = serial.Serial(currentPort[0], timeout = 0.1)
            serialPort.close()
            serialPort.open()
            serialPort.write(b'P')
            response = serialPort.readline().decode().strip()
            if(response == expectedResponse):
                return serialPort
            else:
                print("Not the target, response: %s" % (response))
                serialPort.close()
        except Exception as e:
            print("Unable to Open Port: %s" % (e))
            continue
