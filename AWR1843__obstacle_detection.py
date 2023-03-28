#2023-03-28
#1042_Baker
#Awr1843 Boost
import serial
import time
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui
import matplotlib.pyplot as plt
import numpy

# Change the configuration file named
configFileName = 'ods_default_config.cfg'

CLIport = {}
Dataport = {}
byteBuffer = np.zeros(2 ** 15, dtype='uint8')
byteBufferLength = 0

testroot = []
confirmedroot = []
Z_k_prev = numpy.mat([])

cnt = 1
show = []
changdu = 5


# class clu(object):
#     def __init__(self, x=0, y=0, z=0, xs=0, ys=0, zs=0, frame=0):
#         self.x = x
#         self.y = y
#         self.z = z
#         self.xs = xs
#         self.ys = ys
#         self.zs = zs
#         self.frame = frame


# ------------------------------------------------------------------
#configure the serial ports and send the data from the configuration file to the radar
def serialConfig(configFileName):
    global CLIport
    global Dataport
    # Open the serial ports for the configuration and the data ports
    CLIport = serial.Serial('COM6', 115200)
    Dataport = serial.Serial('COM8', 921600)

    # Read the configuration file and send it to the board
    config = [line.rstrip('\r\n') for line in open(configFileName)]
    for i in config:
        print(i)
        if len(i)>0:
            if i[0] != '%':
                CLIport.write((i + '\n').encode())
                time.sleep(0.1)
                reply = CLIport.read(CLIport.in_waiting).decode()
                print(reply)
    return CLIport, Dataport
# Function to parse the data inside to the configuration file
def parseConfigFile(configFileName):
    configParameters = {}  # Initialize an empty dictionary to store the configuration parameters

    # Read the configuration file and send it to the board
    config = [line.rstrip('\r\n') for line in open(configFileName)]
    for i in config:

        # Split the line
        splitWords = i.split(" ")

        # Hard code the number of antennas, change if other configuration is used
        numRxAnt = 4
        numTxAnt = 3

        # Get the information about the profile configuration
        if "profileCfg" in splitWords[0]:
            startFreq = int(float(splitWords[2]))
            idleTime = int(splitWords[3])
            rampEndTime = float(splitWords[5])
            freqSlopeConst = float(splitWords[8])
            numAdcSamples = int(splitWords[10])
            numAdcSamplesRoundTo2 = 1

            while numAdcSamples > numAdcSamplesRoundTo2:
                numAdcSamplesRoundTo2 = numAdcSamplesRoundTo2 * 2

            digOutSampleRate = int(splitWords[11])

        # Get the information about the frame configuration
        elif "frameCfg" in splitWords[0]:
            chirpStartIdx = int(splitWords[1])
            chirpEndIdx = int(splitWords[2])
            numLoops = int(splitWords[3])
            numFrames = int(splitWords[4])
            framePeriodicity = float(splitWords[5])

    # Combine the read data to obtain the configuration parameters
    numChirpsPerFrame = (chirpEndIdx - chirpStartIdx + 1) * numLoops
    configParameters["numDopplerBins"] = numChirpsPerFrame / numTxAnt
    configParameters["numRangeBins"] = numAdcSamplesRoundTo2
    configParameters["rangeResolutionMeters"] = (3e8 * digOutSampleRate * 1e3) / (
            2 * freqSlopeConst * 1e12 * numAdcSamples)
    configParameters["rangeIdxToMeters"] = (3e8 * digOutSampleRate * 1e3) / (
            2 * freqSlopeConst * 1e12 * configParameters["numRangeBins"])
    configParameters["dopplerResolutionMps"] = 3e8 / (
            2 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 * configParameters["numDopplerBins"] * numTxAnt)
    configParameters["maxRange"] = (300 * 0.9 * digOutSampleRate) / (2 * freqSlopeConst * 1e3)
    configParameters["maxVelocity"] = 3e8 / (4 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 * numTxAnt)

    return configParameters


Dataport = {}
byteBuffer = np.zeros(2**15,dtype = 'uint8')
byteBufferLength = 0

# Funtion to read and parse the incoming data
def readAndParseData18xx(Dataport):
    global byteBuffer, byteBufferLength
    
    # Constants
    OBJ_STRUCT_SIZE_BYTES = 12
    BYTE_VEC_ACC_MAX_SIZE = 2**15
    MMWDEMO_UART_MSG_DETECTEDObjects = 1
    MMWDEMO_UART_MSG_DETECTED_CLUSTERS = 2
    MMWDEMO_UART_MSG_RANGE_PROFILE   = 2
    maxBufferSize = 2**15
    tlvHeaderLengthInBytes = 8
    pointLengthInBytes = 16
    magicWord = [2, 1, 4, 3, 6, 5, 8, 7]
    
    # Initialize variables
    magicOK = 0 # Checks if magic number has been read
    dataOK = 0 # Checks if the data has been read correctly
    frameNumber = 0
    detObj = {}
    detcluster={}
    
    readBuffer = Dataport.read(Dataport.in_waiting)
    byteVec = np.frombuffer(readBuffer, dtype = 'uint8')
    byteCount = len(byteVec)
    
    # Check that the buffer is not full, and then add the data to the buffer
    if (byteBufferLength + byteCount) < maxBufferSize:
        byteBuffer[byteBufferLength:byteBufferLength + byteCount] = byteVec[:byteCount]
        byteBufferLength = byteBufferLength + byteCount
        
    # Check that the buffer has some data
    if byteBufferLength > 16:
        
        # Check for all possible locations of the magic word
        possibleLocs = np.where(byteBuffer == magicWord[0])[0]

        # Confirm that is the beginning of the magic word and store the index in startIdx
        startIdx = []
        for loc in possibleLocs:
            check = byteBuffer[loc:loc+8]
            if np.all(check == magicWord):
                startIdx.append(loc)
               
        # Check that startIdx is not empty
        if startIdx:
            
            # Remove the data before the first start index
            if startIdx[0] > 0 and startIdx[0] < byteBufferLength:
                byteBuffer[:byteBufferLength-startIdx[0]] = byteBuffer[startIdx[0]:byteBufferLength]
                byteBuffer[byteBufferLength-startIdx[0]:] = np.zeros(len(byteBuffer[byteBufferLength-startIdx[0]:]),dtype = 'uint8')
                byteBufferLength = byteBufferLength - startIdx[0]
                
            # Check that there have no errors with the byte buffer length
            if byteBufferLength < 0:
                byteBufferLength = 0
                
            # word array to convert 4 bytes to a 32 bit number
            word = [1, 2**8, 2**16, 2**24]
            
            # Read the total packet length
            totalPacketLen = np.matmul(byteBuffer[12:12+4],word)
            
            # Check that all the packet has been read
            if (byteBufferLength >= totalPacketLen) and (byteBufferLength != 0):
                magicOK = 1
    
    # If magicOK is equal to 1 then process the message
    if magicOK:
        # word array to convert 4 bytes to a 32 bit number
        word = [1, 2**8, 2**16, 2**24]
        
        # Initialize the pointer index
        idX = 0
        
        # Read the header
        magicNumber = byteBuffer[idX:idX+8]
        idX += 8
        version = format(np.matmul(byteBuffer[idX:idX+4],word),'x')
        idX += 4
        totalPacketLen = np.matmul(byteBuffer[idX:idX+4],word)
        idX += 4
        platform = format(np.matmul(byteBuffer[idX:idX+4],word),'x')
        idX += 4
        frameNumber = np.matmul(byteBuffer[idX:idX+4],word)
        idX += 4
        timeCpuCycles = np.matmul(byteBuffer[idX:idX+4],word)
        idX += 4
        numDetectedObj = np.matmul(byteBuffer[idX:idX+4],word)
        idX += 4
        numTLVs = np.matmul(byteBuffer[idX:idX+4],word)
        idX += 4
        subFrameNumber = np.matmul(byteBuffer[idX:idX+4],word)
        idX += 4
        #print(byteBuffer, len(byteBuffer),'\n')
        # Read the TLV messages
        for tlvIdx in range(numTLVs):
            
            # word array to convert 4 bytes to a 32 bit number
            word = [1, 2**8, 2**16, 2**24]

            # Check the header of the TLV message
            tlv_type = np.matmul(byteBuffer[idX:idX+4],word)
            idX += 4
            tlv_length = np.matmul(byteBuffer[idX:idX+4],word)
            idX += 4
            
            # Read the data depending on the TLV message
            # GuiMonitor parameters: 0 1 0 0
            # Radar do not send the detected object array
            if tlv_type == MMWDEMO_UART_MSG_DETECTEDObjects:  
                tlv_numDetobj = np.matmul(byteBuffer[idX:idX+2],word[0: 2])
                idX += 2
                tlv_xyzQFormat = np.matmul(byteBuffer[idX:idX+2],word[0: 2])
                idX += 2
                # Initialize the arrays
                obj_x = np.zeros((numDetectedObj, 2))
                obj_y = np.zeros((numDetectedObj, 2))
                obj_z = np.zeros((numDetectedObj, 2))
                speedIdx = np.zeros((numDetectedObj, 2))
                rangeIdx = np.zeros((numDetectedObj, 2))
                peakVal = np.zeros((numDetectedObj, 2))

                
                for objectNum in range(numDetectedObj):
                    
                    # Read the data for each object
                    speedIdx[objectNum] = byteBuffer[idX:idX + 2]
                    idX += 2
                    obj_x[objectNum] = byteBuffer[idX:idX + 2]
                    idX += 2
                    obj_y[objectNum] = byteBuffer[idX:idX + 2]
                    idX += 2
                    obj_z[objectNum] = byteBuffer[idX:idX + 2]
                    idX += 2
                    rangeIdx[objectNum] = byteBuffer[idX:idX + 2]
                    idX += 2
                    peakVal[objectNum] = byteBuffer[idX:idX + 2]
                    idX += 2
                
                # Store the data in the detObj dictionary
                detObj = {"numObj": numDetectedObj, "x": obj_x, "y": obj_y, "z": obj_z, "velocity":speedIdx}
                dataOK = 1
                # GuiMonitor parameters: 0 1 0 0
                # tlv_type==2
            if tlv_type == MMWDEMO_UART_MSG_DETECTED_CLUSTERS:
                
                tlv_numClusters = np.matmul(byteBuffer[idX:idX+2],word[0: 2])
                idX += 2
                tlv_xyzQFormat = 2**np.matmul(byteBuffer[idX:idX+2],word[0: 2])
                idX += 2

                # Initialize the arrays
                xsize = np.zeros((tlv_numClusters),dtype= np.float)
                ysize = np.zeros((tlv_numClusters),dtype= np.float)
                zsize = np.zeros((tlv_numClusters),dtype= np.float)
                
                clu_x = np.zeros((tlv_numClusters),dtype= np.float)
                clu_y = np.zeros((tlv_numClusters),dtype= np.float)
                clu_z = np.zeros((tlv_numClusters),dtype= np.float)

                # for i in range(tlv_numClusters):
                #     exec('clu_{} = clu()'.format(i))

                for clusterNum in range(tlv_numClusters):
                    
                    # Read the data for each cluster
                    clu_x[clusterNum] = byteBuffer[idX]+byteBuffer[idX+1]*256
                    clu_x[clusterNum] = clu_x[clusterNum] - 65536 if clu_x[clusterNum] > 32767 else clu_x[clusterNum]
                    clu_x[clusterNum] = clu_x[clusterNum]/tlv_xyzQFormat
                    X= clu_x[clusterNum]
                    idX += 2

                    clu_y[clusterNum] = byteBuffer[idX]+byteBuffer[idX+1]*256
                    clu_y[clusterNum] = clu_y[clusterNum] - 65536 if clu_y[clusterNum] > 32767 else clu_y[clusterNum]
                    clu_y[clusterNum] = clu_y[clusterNum]/tlv_xyzQFormat
                    Y= clu_y[clusterNum]
                    idX += 2

                    clu_z[clusterNum] = byteBuffer[idX]+byteBuffer[idX+1]*256
                    clu_z[clusterNum] = clu_z[clusterNum] - 65536 if clu_z[clusterNum] > 32767 else clu_z[clusterNum]
                    clu_z[clusterNum] = clu_z[clusterNum]/tlv_xyzQFormat
                    Z= clu_z[clusterNum]
                    idX += 2

                    xsize[clusterNum] = (byteBuffer[idX]+byteBuffer[idX+1]*256)
                    if xsize[clusterNum] == 0:
                        xsize[clusterNum]+=0.1
                    xsize[clusterNum]=xsize[clusterNum]/tlv_xyzQFormat
                    xsize_center = xsize[clusterNum]/2
                    idX += 2
                    ysize[clusterNum] = (byteBuffer[idX]+byteBuffer[idX+1]*256)
                    if ysize[clusterNum] == 0:
                        ysize[clusterNum]+=0.1
                    ysize[clusterNum]=ysize[clusterNum]/tlv_xyzQFormat
                    ysize_center = ysize[clusterNum]/2
                    idX += 2
                    zsize[clusterNum] = (byteBuffer[idX]+byteBuffer[idX+1]*256)
                    if zsize[clusterNum] == 0:
                        zsize[clusterNum]+=0.1
                    zsize[clusterNum]=zsize[clusterNum]/tlv_xyzQFormat
                    zsize_center = zsize[clusterNum]/2
                    idX += 2

                    pos=[X,Y,Z]
                    size=[xsize_center,ysize_center,zsize_center]
                    pos_array = numpy.array(pos)
                    size_array = numpy.array(size)
                    center=pos_array-size_array

                    # exec('clu_{}.x = {}'.format(clusterNum, clu_x[clusterNum]))
                    # exec('clu_{}.y = {}'.format(clusterNum, clu_y[clusterNum]))
                    # exec('clu_{}.z = {}'.format(clusterNum, clu_z[clusterNum]))
                    # exec('clu_{}.xs = {}'.format(clusterNum, xsize[clusterNum]))
                    # exec('clu_{}.ys = {}'.format(clusterNum, ysize[clusterNum]))
                    # exec('clu_{}.zs = {}'.format(clusterNum, zsize[clusterNum]))
                    # exec('clu_{}.frame = {}'.format(clusterNum, frameNumber))
                    
                    
                # Store the data in the detObj dictionary
                # detcluster = {"numCluster": tlv_numClusters, 
                #               "x": clu_x, "y": clu_y, "z": clu_z, 
                #               "xsize":xsize,"ysize":ysize,"zsize":zsize,"center":center}
                detcluster = {"center":center}
                dataOK = 1
 
        # Remove already processed data
        if idX > 0 and byteBufferLength>idX:
            shiftSize = totalPacketLen
            
                
            byteBuffer[:byteBufferLength - shiftSize] = byteBuffer[shiftSize:byteBufferLength]
            byteBuffer[byteBufferLength - shiftSize:] = np.zeros(len(byteBuffer[byteBufferLength - shiftSize:]),dtype = 'uint8')
            byteBufferLength = byteBufferLength - shiftSize
            
            # Check that there are no errors with the buffer length
            if byteBufferLength < 0:
                byteBufferLength = 0         

    return dataOK, frameNumber, detObj, detcluster

# -----------------------------------------------------------------

# Funtion to update the data and display in the plot
def update():
     
    dataOk = 0
    global detObj
    global detcluster

    # Read and parse the received data
    dataOk, frameNumber, detObj, detcluster= readAndParseData18xx(Dataport)
    
    if dataOk:
       
        for i in detcluster.keys():
            print('{}:{}'.format(i,detcluster[i]))
        print("Fram:",frameNumber)
    return dataOk


# -------------------------    MAIN   -----------------------------------------  

# Configurate the serial port
serialConfig(configFileName)

# Main loop 
detObj = {}  
frameData = {}    
currentIndex = 0
while True:
    try:
        # Update the data and check if the data is okay
        dataOk = update()
        
        if dataOk:
            # Store the current frame into frameData
            frameData[currentIndex] = detObj
            currentIndex += 1
        time.sleep(0.1)
        
    # Stop the program and close everything if Ctrl + c is pressed
    except KeyboardInterrupt:
        print("----------------------Program Terminated----------------------")
        Dataport.close()
        break
