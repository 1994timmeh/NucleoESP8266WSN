from django.shortcuts import HttpResponse
from django.core import serializers

from .models import Node, Signal, CorrMax

import socket
import sys
import _thread



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
    output = serializers.serialize("json", VehicleEstimate.objects.all())
    return HttpResponse(output)
	
def StartTcpClient(request):
	_thread.start_new_thread(tcp_client_thread, ())
	return HttpResponse("client started(probably)")
	
	
	
	
# TODO move this hack elsewhere

def tcp_client_thread():
	# Create a TCP/IP socket
	sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

	server_address = ('192.168.3.1', 8888)
	sock.connect(server_address)

	try:
		
		# Send data
		message = 'DA:[100client]'
		sock.sendall(bytes(message, 'UTF-8'))

		
		while 1:
			data = sock.recv(16)
			if data:
				#split 
				#
				
				#example, process signals
				# send to a processing thread?
				#create new thread to process this?
				#
				c = CorrMax(Node_ID=1, MaxBin=10, MaxValue=1, Timestamp=10001)
				c.save()
				#s = Signal(Node_ID=1, Angle=90, Intensity=1, Timestamp=10000)
				#s.save()

	finally:
		sock.close()
