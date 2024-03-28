# ---Run from machine running the Carla simulation--- #
# ---Called from the cosimManager.py script--- #
# !!! Do not run this standalone. Objects are initialized and their functions called from main.py !!! #


HOST = CONFIG.APOLLO_HOST
PORT = CONFIG.APOLLO_PORT

import pprint
import time
import datetime
import os
import sys
import socket
import numpy as np
import cv2

# Libraries to generate binary headers describing the size of the message payload
import struct
import netstruct

# Module containing the encoder functions to generate and serialize the protoBuf messages for Apollo
from apolloEncodeDecode import *

# Control Command message type to decode the received binary message and read from Prescan
#from protoLib.control_cmd_pb2 import ControlCommand
# The publisher class containing member functions and variables to send data from Prescan to Apollo

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
            now float: current timestamp
            prevMsg float: expected time of the previous msg
            delay float : the expected delay of this message stream
        Returns:

        """
        gap = now - prevMsg
        skips = (gap- gap%delay)/delay #ideally skips is 0
        if skips > 20:
            return prevMsg + delay*(skips)
        else:
            return prevMsg + delay


    # 
    def sendBinMsg(self, data, msgType):
        """ Netstruct protocol function to send packaged data as binary
        
        Args:
            data :
        Returns:

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
        
        Args:

        Returns:

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
        
        Args:

        Returns:
            The message from the server, decoded with self.decodeSimData
        """
        binMsg = self.recvMsg()
        return self.decodeSimData(binMsg)
    
     # Paired function to sendCCMsg from ApolloBridgeServer.py
    def recvMsg(self):
        """ 
        
        Args:

        Returns:

        """
        binMsgLen = self.recvall(4)
        msgLen = struct.unpack('!I', binMsgLen)
        return self.recvall(msgLen[0])

    def recvall(self, count):
        """ 
        
        Args:

        Returns:

        """
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
            hostIP string: The IP of the ads host (server)
            hostPort int: The port of the ads host (server)
        """
        delays = 0.1
        lastMsgTime = 0
        super().__init__(hostIP,hostPort,b'ActorGTPub',delays,lastMsgTime)

    def sendSensorData(self,data,type):
        """ 
        
        Args:

        Returns:

        """
        self.sendBinMsg(data, type)

    # Function to send Simulator data from MATLAB. The arguments are populated from the MATLAB script
    def sendActorGroundTruth(self, dataList):
        """ 
        
        Args:

        Returns:

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
            hostIP string: The IP of the ads host (server)
            hostPort int: The port of the ads host (server)
        """
        delays = {'Gps':0.1, 'IMU':0.01, 'cIMU':0.01, 'BP':0.01, 'INS':0.01, 'Cha':0.01, 'Hea':0.01, 'Pos':0.01 , 'Tf':0.01 }
        nextMsgTime = {'Gps':0.0, 'IMU':0.0, 'cIMU':0.0, 'BP':0.0, 'INS':0.0, 'Cha':0.0, 'Hea':0.0, 'Pos':0.0 , 'Tf':0.0 }
        super().__init__(hostIP,hostPort,b'EgoPub',delays,nextMsgTime)
        
    def sendEgoData(self, dataList):
        """ 
        
        Args:

        Returns:

        """
        # print("Sent")
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
        """Generic Bridge Client

        Args:
            hostIP string: The IP of the ads host (server)
            hostPort int: The port of the ads host (server)
        """
        
        super().__init__(hostIP,hostPort,b'ImgPub')

    # Function to send Simulator data from MATLAB. The arguments are populated from the MATLAB script
    def sendcImgData(self, cimgData):
        """ 
        
        Args:

        Returns:

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
        """ 
        
        Args:

        Returns:

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
        """ 
        
        Args:

        Returns:

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
        ccmsg = ControlCommand()
        ccmsg.ParseFromString(binMsg)
        #gear_location = ccmsg.gear_location
        return [ccmsg.throttle, ccmsg.brake, ccmsg.steering_target, ccmsg.steering_rate]
    
