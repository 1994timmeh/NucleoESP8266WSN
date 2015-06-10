from django.shortcuts import HttpResponse
from django.core import serializers

from .models import Node, Signal, CorrMax, Frame, VehicleEstimate

import socket
import sys
import _thread
import time
import datetime
import math
import base64



import csse4011_api.processing



#views here

def index(request):
    return HttpResponse("This is the index.")

def Nodes(request):
    output = serializers.serialize("json", Node.objects.all())
    return HttpResponse(output)
    
    
def Signals(request, timeStamp):
    output = serializers.serialize("json", Signal.objects.all())
    return HttpResponse(output)
    
def CorrMaxs(request, timeStamp):
    output = serializers.serialize("json", CorrMax.objects.all())
    return HttpResponse(output)
    
def VehicleEstimates(request, timeStamp):
    output = serializers.serialize("json", VehicleEstimate.objects.filter(FrameNum__gt = timeStamp))
    return HttpResponse(output)
    
def StartTcpClient(request):
    _thread.start_new_thread(tcp_client_thread, ())
    return HttpResponse("client started(probably)")
    
    
    



def handleFrame(frameNum, frameDataDecoded, f, node_ID):
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
    
#    if frameData[1] > 2147483647: #this is a hack
#        maxBin = -1*(2147483647*2 - frameData[1])

	if frameData[1] & 0x80000000:
		 maxBin = -1*(frameData[1] & 0x7FFFFFFF)
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

    print("Node_ID: " + str(node_ID))


    # log to file
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
        str(kurtosis) + ',' +
        str(frameNumNode) + ',' +
        str(node_ID) +
        '\n')

    # save to database
    #   frame = Frame(MaxBin = maxBin, 
    # MaxFrequencies = ','.join(maxFrequencies),
    # MaxValue = maxValue,
    # Power = power,
    # Mean = mean,
    # Variance = variance,
    # Skew = skew,
    # Kurtosis = kurtosis,
    # Node_ID = node_ID)
    #   frame.save()
    # nodemeasurement = processing.Measurement(int(maxBin), maxFrequencies, maxValue, power, mean, variance, skew, kurtosis)
    # print("<processing here> Node_ID: " + str(node_ID))
    # return nodemeasurement


# TODO move this hack elsewhere
def tcp_client_thread():
        
    f = open('./csse4011_api/logs/framesLog_' + str(int(time.time())) + '.csv','w')

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    server_addr = ('192.168.1.1', 8888)
    sock.connect(server_addr)
    sock.send(bytes("DA:[100]", 'UTF-8'))

    node0measurements = []
    node1measurements = []

    #setup processing environment
    nodes = Node.objects.all()
    # node1 = processing.Node(nodes[0].latitude, node[0].longitude, 43, 0.3, 48e3)
    # node2 = processing.Node(nodes[1].latitude, node[1].longitude, 43, 0.3, 48e3)

    # deployment = processing.Deployment(node1, node2,  50)
    # deployment.resetKalman(nodes[0].latitude, nodes[1].longitude, 0, 0)

    lastFrameComplete = -1;

    while(True):
        data = sock.recv(1000)
        node_ID = 0 #int(data[5])
        print("----------------------------------------------------------")
        print(data)
        print("-----------------------------------------------------------")
        data = data[7:-1]
        print(data)
        print("-----------------------------------------------------------")
        #node = int(data[5])
        #print("NODE_ID: " + str(node))
        decoded = "failed"
        try:
        	decoded = base64.b64decode(data)
        	print("endoded length: " + str(len(data)))
        except:
        	print("##################DECODE FAILED###############")
        	print("endoded length: " + str(len(data)))
        	print("decoded length: " + str(len(decoded)))
        	print(decoded)
        	continue
        frameData = []
        frameNum = 0
        word = 0

            
            
        for a in range(0, int(len(decoded)/51)): 
            if node_ID == 0:
                node0measurements.append(handleFrame(a, decoded[51*a:51*a+51], f, node_ID))
            elif node_ID == 1:
                node1measurements.append(handleFrame(a, decoded[51*a:51*a+51], f, node_ID))
            else:
                print("INVALID NODE_ID: " + str(node_ID))
            

        # find two  
        print("node0measurements - length: " + str(len(node0measurements)))
        print("node1measurements - length: " + str(len(node1measurements)))

        #find next frame to process in node0measuermens
        #find comparable frame in node1measurements
        	#if can find comparable
        		#process
        	#else
        		#lastFrameComplete++
        


        
        
    f.close()
