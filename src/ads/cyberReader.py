# ---Dependency for and called from bridgeServer.py--- #

# Python class for reading from CyberRT

from ctypes import sizeof
from cyber.python.cyber_py3 import cyber
import time
import datetime
import os
import threading


# Generated protoBuf message classes
from modules.common_msgs.control_msgs.control_cmd_pb2 import ControlCommand

# Class definition for cyberReader
class cReader():
    def __init__(self):
        # CyberRT is initialized
        cyber.init()
        self.cNode = None
        self.ccMsg = None
        self.ccBinMsg = None
        self.binMsg = None
        self.msgID = None
        self.msgLen = None
        self.ccReader = None
        self.dataAvailableToSend = False
        self.ccBinList = []

        # A threading lock function is initialized, to copy data from the CyberRT callback function
        self.mutex = threading.Lock()

    # To read messages from CyberRT, a callback function must be specified
    def callback(self, data):
        # Thread locked
        self.mutex.acquire()
        self.ccMsg = data
        self.ccBinMsg = self.encodeBinMsg(self.ccMsg)
        self.ccBinList.append(self.ccBinMsg)
        self.dataAvailableToSend = True
        # Thread unlocked
        self.mutex.release()

    # Function to encode the CCmsg to binary (for sending to client)
    def encodeBinMsg(self, msg):
        binMsg = msg.SerializeToString()
        return binMsg

    # Function to create a CyberRT reader and assign the callback defined above to it
    def makeReader(self):
        # CyberRT node is created
        self.cNode = cyber.Node("ApolloCarlaReader")
        self.ccReader = self.cNode.create_reader("/apollo/control", ControlCommand, self.callback)

    # Thread locked function to read from the ccBinMsg variable and ccBinMsgList to send to client
    def readMsgList(self, clearFlag = True):
        # Thread locked
        self.mutex.acquire()
        ccBinList = self.ccBinList[:]
        ccBinMsg = self.ccBinMsg
        ccMsg = self.ccMsg
        if clearFlag is True:
            self.ccBinList = []
            self.ccBinMsg = None
        # Thread unlocked
        self.mutex.release()
        return ccBinList, ccBinMsg, ccMsg
