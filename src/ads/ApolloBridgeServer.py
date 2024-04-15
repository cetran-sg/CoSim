# ---Run inside the Apollo Docker container--- #
# ---Ensure the HOST and PORT set in main() matches the client--- #

import os
import struct
import netstruct
import sys
import time
from datetime import datetime
import socket
import multiprocessing

from cyber.python.cyber_py3 import cyber

# Classes for decoding binary messages and writing to CyberRT
from cyberWriter import *
from cyberReader import *


from modules.common_msgs.sensor_msgs.sensor_image_pb2 import CompressedImage,Image
from modules.common_msgs.control_msgs.control_cmd_pb2 import ControlCommand
from modules.common_msgs.sensor_msgs.gnss_best_pose_pb2 import GnssBestPose
from modules.common_msgs.sensor_msgs.heading_pb2 import Heading
from modules.common_msgs.localization_msgs.gps_pb2 import Gps
from modules.common_msgs.sensor_msgs.ins_pb2 import InsStat
from modules.common_msgs.chassis_msgs.chassis_pb2 import Chassis
from modules.common_msgs.localization_msgs.imu_pb2 import CorrectedImu
from modules.common_msgs.sensor_msgs.imu_pb2 import Imu
from modules.common_msgs.localization_msgs.localization_pb2 import LocalizationEstimate
from modules.common_msgs.transform_msgs.transform_pb2 import TransformStampeds
from modules.common_msgs.sensor_msgs.pointcloud_pb2 import PointCloud
from modules.common_msgs.perception_msgs.perception_obstacle_pb2 import PerceptionObstacles, PerceptionObstacle

def ccHandler(server, conn, addr, prID):
    """ Handler for control command messages to be published to the simulator

    """

    ccBinList = None
    # CyberReader instance is created
    print("Create reader")
    cyberReader = cReader()
    cyberReader.makeReader()
    print("Reader created")

    while True:
        if cyberReader.dataAvailableToSend is True:
            # The below function is thread locked with mutex
            # This is to prevent memory corruption
            ccBinList, ccBinMsg, ccMsg = cyberReader.readMsgList()
            if ccBinMsg:
                item = ccBinMsg
                Server.sendCCMsg(conn, item)
        else:
            pass

def connHandler(server, conn, addr, prID, initmsgType):
    """ Connection handler for subscribed messages from the simulator
    
    """

    try:
        print("-- Connected to client from %r -- ", addr)
        # Get the initial message type and assign task to the handler
        if initmsgType == 'ActorGTPub':
            writerConfDict = {'PO':("/apollo/perception/obstacles", PerceptionObstacles, 2)}
            writer = CyberWriter( "GTWriter", writerConfDict)
            writer.makeWriter()
            msgType = 'Start'
            
        elif initmsgType == 'EgoPub':
            # CyberWriter instance is created and the logger is attached to it
            writerConfDict = {'Gps':("/apollo/sensor/gnss/odometry", Gps, 1),
            'IMU':("/apollo/sensor/gnss/imu", Imu, 3),
            'cIMU':("/apollo/sensor/gnss/corrected_imu", CorrectedImu, 4),
            'BP':("/apollo/sensor/gnss/best_pose", GnssBestPose, 5),
            'INS':("/apollo/sensor/gnss/ins_stat", InsStat, 6),
            'Cha':("/apollo/canbus/chassis", Chassis, 7),
            'Hea':("/apollo/sensor/gnss/heading", Heading, 8),
            'Pos':("/apollo/localization/pose",LocalizationEstimate,9),
            'Tf':("/tf",TransformStampeds,10)}
            writer = CyberWriter( "EgoWriter",writerConfDict)
            writer.makeWriter()
            msgType = 'Start'

        elif initmsgType == 'ImgPub':
            # CyberWriter instance is created
            writerConfDict = {"Img": ("/apollo/sensor/camera/front_6mm/image",Image,10) , "cImg":("/apollo/sensor/camera/front_6mm/image/compressed",CompressedImage,11)}
            writer = CyberWriter("ImgWriter" , writerConfDict)
            writer.makeWriter()
            msgType = 'Start'

        elif initmsgType == 'PCPub':

            # CyberWriter instance is created
            #pcCount = initmsgType[-1]
            writerConfDict = {"PC": ("/apollo/sensor/lidar128/compensator/PointCloud2",PointCloud,12)}
            writer = CyberWriter("PCWriter", writerConfDict)
            writer.makeWriter()
            msgType = 'Start'

        # Received messages are written to CyberRT until the 'Stop' message is received
        while msgType != 'Stop':
            binMsg, msgType, msgLen = server.recvBinMsg(conn)
            writer.writeMessage(binMsg, msgType, msgLen)
    except:
        print("Problem handling request")
    finally:
        conn.close()
        cyber.shutdown()
        sys.exit()


class Server():
    def __init__(self, host, port):
        """ Main bridge server class file with host and port definitions
        
        Args:
            host string: IP of the bridge server, typically '127.0.0.1' (but if Carla is running on another machine, give an IP on the same subnet)
            port int: Port of the bridge server, typically 9999
        """
        self.host = host
        self.port = port
        self.socket = None

    def start(self):
        """ Start the server, accept connections from clients and assign them to handlers (each on a different process)
        """
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        self.socket.bind((self.host, self.port))
        self.socket.listen(5)
        print("\n-- Bridge Server started. Awaiting client --\n")
        prID = 1
        while True:
            conn, address = self.socket.accept()
            print("\n-- Client connected --\n")
            initbinMsg, initmsgType, initmsgLen = self.recvBinMsg(conn)
            if initmsgType == "Subscriber":
                ccProcess = multiprocessing.Process(target=ccHandler, args=(self,conn, address, prID))
                ccProcess.daemon = True
                ccProcess.start()
            else:
                process = multiprocessing.Process(target=connHandler, args=(self,conn, address, prID, initmsgType))
                process.daemon = True
                process.start()
            # Each process is assigned as a daemon of the main process
            # Killing the main process using 'fuser -k 9999/tcp' will also kill the child processes
            print("\n-- Started handler process PID-" + str(prID) + " --\n")
            prID += 1


    
    # Netstruct protocol function to receive packaged binary message
    def recvBinMsg(self,sock):
        binHLen = self.recvall(sock, 4)
        headerLen = struct.unpack('!I', binHLen)
        binHeader = self.recvall(sock, headerLen[0])
        msgHeader = netstruct.unpack(b"!Ib$", binHeader)
        msgLen = msgHeader[0]
        msgType = msgHeader[1].decode('ascii')
        return self.recvall(sock, msgLen), msgType, msgLen

    # Netstruct protocol function to receive packaged binary message
    # Called from recvBinMsg
    def recvall(self,sock, count):
        buf = b''
        while count:
            newbuf = sock.recv(count)
            # print('Count,len(newbuf):', count, len(newbuf))
            if not newbuf: return None
            buf += newbuf
            count -= len(newbuf)
        return buf

    # Netstruct protocol function to send packaged binary message
    def sendBinMsg(self,sock, data, msgType):
        dataLen = len(data)
        header = netstruct.pack(b"!Ib$", dataLen, msgType)
        headerLen = len(header)

        sock.sendall(struct.pack('!I', headerLen))
        sock.sendall(header)
        sock.sendall(data)

    # Netstruct protocol function to send only ControlCommand message
    # A paired receive function exists on the client side
    # This is because the Control Command message type is known
    # and only the Control Command is sent back from the server
    def sendCCMsg(sock, data):
        dataLen = len(data)
        sock.sendall(struct.pack('!I', dataLen))
        sock.sendall(data)

# Main function, HOST and PORT are declared and the bridgeServer object is initialized and started
def main():
    HOST = '127.0.0.1' # Standard loopback interface address (localhost) - change to IP of Apollo PC if any issues
    PORT = 9999        # Port to listen on (non-privileged ports are > 1023)
    bridgeServer = Server(HOST, PORT)
    bridgeServer.start()

    cyber.shutdown()
    sys.exit()


if __name__ == '__main__':
    main()