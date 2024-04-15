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
        ''' Initialize CyberReader object
        
        '''
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
        ''' Callback function to get control command messages from CyberRT
        
        '''
        # Thread locked
        self.mutex.acquire()
        self.ccMsg = data
        self.ccBinMsg = self.encodeBinMsg(self.ccMsg)
        self.ccBinList.append(self.ccBinMsg)
        self.dataAvailableToSend = True
        # Thread unlocked
        self.mutex.release()

    def encodeBinMsg(self, msg):
        ''' Function to encode the control commands to binary for sending to the simulator
        
            Args:
                msg : Control command message
        '''
        binMsg = msg.SerializeToString()
        return binMsg

    def makeReader(self):
        ''' Function to create a CyberRT reader and assign the previously defined callback function to it
        
        '''
        # CyberRT node is created
        self.cNode = cyber.Node("ApolloCarlaReader")
        self.ccReader = self.cNode.create_reader("/apollo/control", ControlCommand, self.callback)

    def readMsgList(self, clearFlag = True):
        ''' Thread locked function to read the control commands and control command list before sending to the simulator

            Args:
                clearFlag bool : Flag to indicate if the backlog of contol commands have been cleared or not
        '''
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