import socket
import sys
import base64
import datetime
import math


# f = open('frameSave','w')
#     f.write('hi there\n') # python will convert \n to os.linesep
#     f.close()

def handleFrame(frameNum, frameData):
    frameData = []
    word = 0
    for i in range(0, 51):
        print(str(i) + ": " + str(decoded[i]))
        if i  == 0:
            frameData.append(decoded[i])
        else:
            if (i-1) % 4 == 0 and i > 1:
                frameData.append(word)
                word = 0;
    
            word += decoded[i] << ((i-1) % 4)*8
    frameData.append(word)    
        
    for a in range(0, 14):
        print("Frame: " + str(frameNum) +" index: " + str(a) + "  -  " + str(frameData[a]))
            
            
    validFrame = frameData[0]
    print("validFrame: " + str(frameData[0]));
    
    if frameData[1] > 2147483647: #this is a hack
        maxBin = 2147483647*2 - frameData[1]
    else:
        maxBin = frameData[1]
    print("maxBin: " + str(maxBin));
    
    maxFrequencies = []
    word = 0
    for b in range(2, 2+5):
        maxFrequencies.append(frameData[b]/10000.0)
        print("maxFrequencies[" + str(b-2) + "]: " + str(frameData[b]/10000.0))
    
    maxValue = frameData[7]/10000.0
    print("maxValue: " + str(maxValue))
    
    power = frameData[8]/10000.0
    print("power: " + str(power))
     
    mean = frameData[9]/10000.0
    print("mean: " + str(mean))
     
    variance = frameData[10]/10000.0
    print("variance: " + str(variance))
     
    skew = frameData[11]/10000.0
    print("skew: " + str(skew))
     
    kurtosis = frameData[12]/10000.0
    print("kurtosis: " + str(kurtosis))
    
    frameNumNode = frameData[13]
    print("frameNumNode: " + str(frameNumNode))
     # you can omit in most cases as the destructor will call if
#     for c in range(8, 8+6):
#         sign = 0x80000000 & frameData[c]
#         exponent = 0x7F800000 & frameData[c]
#         fraction = 0x000



sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

server_addr = ('192.168.2.16', 8888)
sock.connect(server_addr)
sock.send(bytes("DA:[100]", 'UTF-8'))



while(True):
    data = sock.recv(1000)
    print(datetime.datetime.now())
    #node = int(data[5])
    #print("NODE_ID: " + str(node))
    decoded = base64.b64decode(data)
    print(decoded)
    frameData = []
    frameNum = 0
    word = 0
    print("Bytes received: " + str(len(decoded)))

        
        
    for a in range(0, int(len(decoded)/51)): 
        handleFrame(a, decoded[51*a:51*a+51])
        
    #     for i in range(0, 4):
    #         if
    #         maxFrequencies.append(object)
    #     
    #     
    #     
    
    
