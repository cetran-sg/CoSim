# ---Dependency for and called from bridgeServer.py--- #

# Python class for decoding binary messages and writing to CyberRT

from cyber.python.cyber_py3 import cyber

import time
from datetime import datetime
import os

# Generated protoBuf message classes


class CyberWriter():
    def __init__(self, nodeName ,msgDict):
        # CyberRT is initialized
        cyber.init()
        self.cNode = None
        
        self.writerDict = dict()
        self.msgDict = msgDict
        self.nodeName = nodeName

    # Function to create CyberRT writers for different message topics
    def makeWriter(self):
        # CyberRT node is created
        print("creating node ",self.nodeName)
        self.cNode = cyber.Node(self.nodeName)
        print("done ",self.nodeName)

        for key in self.msgDict.keys():
            self.writerDict[key] = self.cNode.create_writer(self.msgDict[key][0],self.msgDict[key][1],self.msgDict[key][2])

    # Function which writes the actual messages using the above created writers
    def writeMessage(self,binMsg, msgType, msgLen):
        # print('img')
        protoMsg = self.msgDict[msgType][1]()
        protoMsg.ParseFromString(binMsg)
        self.writerDict[msgType].write(protoMsg)

