# ---Dependency for and called from bridgeServer.py--- #

# Python class for decoding binary messages and writing to CyberRT

from cyber.python.cyber_py3 import cyber

import time
from datetime import datetime
import os

# Generated protoBuf message classes

class CyberWriter():
    def __init__(self, nodeName, msgDict):
        ''' Initialize CyberWriter object
            
            Args:
                nodeName string : Name of the CyberRT node to write the message into
                msgDict dict : Dictionary containing the message channel and type of message
        '''
        cyber.init()
        self.cNode = None
        
        self.writerDict = dict()
        self.msgDict = msgDict
        self.nodeName = nodeName

    # Function to create CyberRT writers for different message topics
    def makeWriter(self):
        ''' Create a CyberRT node and create a writer to it
        '''
        # CyberRT node is created
        print("Creating node: ",self.nodeName)
        self.cNode = cyber.Node(self.nodeName)
        print("Node created: ",self.nodeName)

        for key in self.msgDict.keys():
            self.writerDict[key] = self.cNode.create_writer(self.msgDict[key][0],self.msgDict[key][1],self.msgDict[key][2])

    # Function which writes the actual messages using the above created writers
    def writeMessage(self,binMsg, msgType, msgLen):
        ''' Write the binary message into the CyberRT node using the previously created writer

            Args:
                binMsg bytestring : Bytestring message received from the simulator
                msgType : Message type from the dictionary which stores all message channels and types
                msgLen int : Length of the message
        
        '''
        protoMsg = self.msgDict[msgType][1]()
        protoMsg.ParseFromString(binMsg)
        self.writerDict[msgType].write(protoMsg)

