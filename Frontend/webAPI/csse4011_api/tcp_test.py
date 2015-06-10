import socket
import sys
import base64
import datetime
import math
import time


f = open('./logs/framesLog_' + str(int(time.time())) + '.csv','w')

def handleFrame(frameNum, frameDataDecoded):
    frameData = []
    word = 0
    for i in range(0, 51):
        print(str(i) + ": " + str(frameDataDecoded[i]))
        if i  == 0:
            frameData.append(frameDataDecoded[i])
        else:
            if (i-1) % 4 == 0 and i > 1:
                frameData.append(word)
                word = 0;
    
            word += frameDataDecoded[i] << ((i-1) % 4)*8
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

    f.write(str(maxBin) + ',' +
        str(maxFrequencies[0]) + ',' +
        str(maxFrequencies[1]) + ',' +
        str(maxFrequencies[2]) + ',' +
        str(maxFrequencies[3]) + ',' +
        str(maxFrequencies[4]) + ',' +
        str(maxValue) + ',' +
        str(power) + ',' +
        str(mean) + ',' +
        str(variance) + ',' +
        str(skew) + ',' +
        str(kurtosis) +
          '\n')



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
    
    
f.close()