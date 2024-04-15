# Objects here are initialized and their functions called from cosimManager.py #
import config as CONFIG

HOST = CONFIG.APOLLO_HOST
PORT = CONFIG.APOLLO_PORT

import sys
import socket
import numpy as np
import cv2

# Libraries to generate binary headers describing the size of the message payload
import struct
import netstruct

# Module containing the encoder functions to generate and serialize the protoBuf messages for Apollo
from apolloEncode import *

# Control command message type, to be decoded from Apollo and written into Carla
from modules.common_msgs.control_msgs.control_cmd_pb2 import ControlCommand

class BridgeClient():
    def __init__(self,hostIP,hostPort,msgString,delay = None,nextMsgTime = None):
        """Generic Bridge Client

        Args:
            hostIP string: The IP of the ads host (server)
            hostPort int: The port of the ads host (server)
            msgString bstring: string ID of the client
            delay float: time delay between each message
            nextMsgTime float: store the expected timestamp of the next message
        """
        self.server_port = (hostIP, hostPort)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # self.delays = dict()
        # self.lastMsgTime = dict()
        self.msgString = msgString
        self.delays = delay
        self.nextMsgTime = nextMsgTime
    
    def connectToServer(self):
        """ Initiate connection
        

        """
        try:
            self.sock.connect(self.server_port)
        except:
            print('Publisher:Failed to connect to the server. ')
            sys.exit()

    def computeNextMsgTime(self,now, prevMsg,delay):
        """ When a message is sent, determines the timestamp of the next message and wheter to skip frames in case of excessive accumulation

        Args:
            now float: Current timestamp
            prevMsg float: Expected time of the previous message
            delay float : Expected delay of this message stream
        Returns:

        """
        gap = now - prevMsg
        skips = (gap - gap%delay)/delay #ideally skips is 0
        if skips > 20:
            return prevMsg + delay*(skips)
        else:
            return prevMsg + delay

    def sendBinMsg(self, data, msgType):
        """ Netstruct protocol function to package and send binary data
        
        Args:
            data bytestring : The data to send
            msgType string : Message type 

        """
        # print("msgstring: " + str(self.msgString))
        dataLen = len(data)
        header = netstruct.pack(b"!Ib$", dataLen, msgType)
        headerLen = len(header)

        self.sock.sendall(struct.pack('!I', headerLen))
        self.sock.sendall(header)
        self.sock.sendall(data)
    
    def register(self):
        """ Register a client object to the server

        """
        print(self.msgString)
        self.sendBinMsg(b'000', self.msgString)

    def closeConnection(self):
        """ Close the connection to the server

        """
        self.msgString = b'Stop'
        self.sendBinMsg(b'000', self.msgString)
        self.sock.close()
    
    def decodeSimData(self,binMsg):
        """ Decodes and return the message received from the server, to be customized based on the client role.

        """
        return None

    def recvSimData(self):
        """ Function to receive messages from the server

        Returns:
            The message from the server, decoded with self.decodeSimData
        """
        binMsg = self.recvMsg()
        return self.decodeSimData(binMsg)
    
     # Paired function to sendCCMsg from ApolloBridgeServer.py
    def recvMsg(self):
        """ Netstruct function to receive message from apolloBridgeServer

        """
        binMsgLen = self.recvall(4)
        msgLen = struct.unpack('!I', binMsgLen)
        return self.recvall(msgLen[0])

    def recvall(self, count):
        buf = b''
        while count:
            newbuf = self.sock.recv(count)
            if not newbuf: return None
            buf += newbuf
            count -= len(newbuf)
        return buf


class ApolloBridgeClient_ActorGTPublisher(BridgeClient):
    def __init__(self,hostIP,hostPort):
        """Generic Bridge Client

        Args:
            hostIP string: The IP of the ADS host (server)
            hostPort int: The port of the ADS host (server)
        """
        delays = 0.1
        lastMsgTime = 0
        super().__init__(hostIP,hostPort,b'ActorGTPub',delays,lastMsgTime)

    def sendSensorData(self,data,type):
        """ Wrapper function to send sensor data using the netstruct protocol
        
        Args:
            data bytestring : Sensor data as a bytestring
            type string : String specifying type of sensor data

        """
        self.sendBinMsg(data, type)

    def sendActorGroundTruth(self, dataList):
        """ Function to send actor ground truth data from the simulator 
        
        Args:
            dataList tuple : Contains all parameters regarding actors in the simulation frame

        """
        (t, seq, objCount, objID, objType, objHeading, objLat, objLon, objutmx, objutmy, objVelX, objVelY, BBx, BBy,BBz) = dataList
        # print(dataList)
        # Messages are encoded using the protoBuf message format expected from Apollo       
        if t > (self.nextMsgTime):  
            self.nextMsgTime = t + self.delays
            if objCount is None or objCount == 0:
                # If no perception obstacles are present, an empty PO message is created
                PO = encode_POmsgs(None, None, None, None, None, None, None, None, None, None, None, None, None, t,seq)
            else:
                # Else, the perception obstacle paramters are used to created the PO message
                PO = encode_POmsgs(objCount, objID, objType, objHeading, objLat, objLon, objutmx, objutmy, objVelX, objVelY, BBx, BBy, BBz, t, seq)
            self.sendBinMsg(PO, b'PO')


class ApolloBridgeClient_EgoPublisher(BridgeClient):
    def __init__(self,hostIP,hostPort):
        """Generic Bridge Client

        Args:
            hostIP string: The IP of the ADS host (server)
            hostPort int: The port of the ADS host (server)
        """
        delays = {'Gps':0.1, 'IMU':0.01, 'cIMU':0.01, 'BP':0.01, 'INS':0.01, 'Cha':0.01, 'Hea':0.01, 'Pos':0.01 , 'Tf':0.01 }
        nextMsgTime = {'Gps':0.0, 'IMU':0.0, 'cIMU':0.0, 'BP':0.0, 'INS':0.0, 'Cha':0.0, 'Hea':0.0, 'Pos':0.0 , 'Tf':0.0 }
        super().__init__(hostIP,hostPort,b'EgoPub',delays,nextMsgTime)
        
    def sendEgoData(self, dataList):
        """ Function to send ego vehicle localization and other data to the ADS.
        Some of the localization messages are not sent, as specified by the 'enableLocMsgs' flag.
        This is because Apollo does not need all the localization messages to drive.
        
        Args:
            dataList tuple : Contains all parameters regarding the ego in the simulation frame

        """
        enableLocMsgs = False
        (t, seq, lat, lon, utmx, utmy, qw, qx, qy, qz, accX,accY,accZ,avelX,avelY,avelZ,odo,eulx,euly,eulz,vel,throttle,brake,steering,velx, vely, heading)= dataList

        if t > (self.nextMsgTime["Gps"]):  
            self.nextMsgTime["Gps"] = self.computeNextMsgTime(t,self.nextMsgTime["Gps"],self.delays["Gps"])            
            Gps = encode_GPSmsg(lat, lon, utmx, utmy, qw, qx, qy, qz, velx, vely, heading, t, seq)
            self.sendBinMsg(Gps, b'Gps')

        if enableLocMsgs and t > (self.nextMsgTime["IMU"]):  
            self.nextMsgTime["IMU"] = self.computeNextMsgTime(t,self.nextMsgTime["IMU"],self.delays["IMU"])
            IMU = encode_IMUmsgs(avelX,avelY,avelZ, accX, accY, accZ, t, seq)
            self.sendBinMsg(IMU, b'IMU')
        
        if enableLocMsgs and t > (self.nextMsgTime["cIMU"]):  
            self.nextMsgTime["cIMU"] = self.computeNextMsgTime(t,self.nextMsgTime["cIMU"],self.delays["cIMU"])
            cIMU = encode_corrIMUmsgs(avelX,avelY,avelZ,eulx,euly,eulz,accX,accY,accZ,heading,t)
            self.sendBinMsg(cIMU, b'cIMU')

        if enableLocMsgs and t > (self.nextMsgTime["BP"]):  
            self.nextMsgTime["BP"] = self.computeNextMsgTime(t,self.nextMsgTime["BP"],self.delays["BP"])
            BP = encode_BPmsgs(lat, lon, utmx, utmy, t, seq)
            self.sendBinMsg(BP, b'BP')

        if enableLocMsgs and t > (self.nextMsgTime["INS"]):  
            self.nextMsgTime["INS"] = self.computeNextMsgTime(t,self.nextMsgTime["INS"],self.delays["INS"])
            INS = encode_INSmsgs(t, seq)
            self.sendBinMsg(INS, b'INS')

        if  t > (self.nextMsgTime["Cha"]):  
            self.nextMsgTime["Cha"] = self.computeNextMsgTime(t,self.nextMsgTime["Cha"],self.delays["Cha"])
            Cha = encode_Chamsgs(t,lat,lon,utmx,utmy,odo,heading,vel,throttle,brake,steering)
            self.sendBinMsg(Cha, b'Cha')

        if t > (self.nextMsgTime["Hea"]):  
            self.nextMsgTime["Hea"] = self.computeNextMsgTime(t,self.nextMsgTime["Hea"],self.delays["Hea"])
            Hea = encode_Heamsgs(t,seq,heading)
            self.sendBinMsg(Hea, b'Hea')

        if (not enableLocMsgs) and t > (self.nextMsgTime["Pos"]):  
            self.nextMsgTime["Pos"] = self.computeNextMsgTime(t,self.nextMsgTime["Pos"],self.delays["Pos"])

            Pos = encode_Posemsgs(lat,lon,utmx,utmy,qw,qx,qy,qz,velx,vely,heading,avelX,avelY,avelZ, accX, accY, accZ,t,seq)
            self.sendBinMsg(Pos, b'Pos')
        
        if (not enableLocMsgs) and t > (self.nextMsgTime["Tf"]):  
            self.nextMsgTime["Tf"] = self.computeNextMsgTime(t,self.nextMsgTime["Tf"],self.delays["Tf"])
            Tf = encode_Tfmsgs(utmx,utmy,eulx,euly,eulz,t,seq)
            self.sendBinMsg(Tf, b'Tf')



class ApolloBridgeClient_ImgPublisher(BridgeClient):
    def __init__(self,hostIP,hostPort):
        """ Generic Bridge Client

        Args:
            hostIP string: The IP of the ads host (server)
            hostPort int: The port of the ads host (server)
        """
        
        super().__init__(hostIP,hostPort,b'ImgPub')

    # Function to send Simulator data from MATLAB. The arguments are populated from the MATLAB script
    def sendcImgData(self, cimgData):
        """ Function to send compressed image data from simulator to ADS
        
        Args:
            cimgData : Compressed image data from the simulator

        """
        # Messages are encoded using the protoBuf message format expected from Apollo
        cimgData =  np.ndarray(
        shape=(cimgData.height, cimgData.width, 4),
        dtype=np.uint8, buffer=cimgData.raw_data)
        #print(img)
        #rgbimg = cimgData[:,:, [2,1, 0,3]]
        rgbimg = cimgData[:,:, [ 0,1,2,3]]
        

        #img = PILImage.fromarray(img, 'RGBA')
        rgbimg = cv2.imencode('.jpg', rgbimg)[1].tostring()
        cImg = encode_CompressedIMG(rgbimg, 0)
        self.sendBinMsg(cImg, b'cImg')

    def sendImgData(self, imgData):
        """ Function to send image data from simulator to ADS
        
        Args:
            imgData : Image data from the simulator

        """

        # Messages are encoded using the protoBuf message format expected from Apollo 
        imgData =  np.ndarray(
        shape=(imgData.height, imgData.width, 4),
        dtype=np.uint8, buffer=imgData.raw_data)
        #print(img)
        rgbimg = imgData[:,:, [2,1, 0,3]]
        

        #img = PILImage.fromarray(img, 'RGBA')
        rgbimg = cv2.imencode('.jpg', rgbimg)[1].tostring()
        Img = encode_IMG(rgbimg, 0)
        self.sendBinMsg(Img, b'Img')


class ApolloBridgeClient_PCPublisher(BridgeClient):
    def __init__(self,hostIP,hostPort):
        """Generic Bridge Client

        Args:
            hostIP string: The IP of the ads host (server)
            hostPort int: The port of the ads host (server)
        """
        super().__init__(hostIP,hostPort,b'PCPub')


    # Function to send Simulator data from MATLAB. The arguments are populated from the MATLAB script
    def sendPCData(self, PCData):
        """ Function to send pointcloud data from simulator to ADS
        
        Args:
            cimgData : Compressed image data from the simulator

        """
        # Messages are encoded using the protoBuf message format expected from Apollo
        pointCloudData = encode_PC(PCData, 0)
        self.sendBinMsg(pointCloudData, b'PC')


class ApolloBridgeClient_Subscriber(BridgeClient):
    def __init__(self,hostIP,hostPort):
        """Generic Bridge Client

        Args:
            hostIP string: The IP of the ads host (server)
            hostPort int: The port of the ads host (server)
        """
        super().__init__(hostIP,hostPort,b'Subscriber')

    def decodeSimData(self,binMsg):
        ''' Function to decode the control command data received from the ADS

            Args:
                binMsg bytestring : Binary message from the ADS to be decoded
        
        '''
        ccmsg = ControlCommand()
        ccmsg.ParseFromString(binMsg)
        #gear_location = ccmsg.gear_location
        return [ccmsg.throttle, ccmsg.brake, ccmsg.steering_target, ccmsg.steering_rate]