import socket
import sys
import base64
import datetime
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

server_addr = ('192.168.2.16', 8888)
sock.connect(server_addr)
sock.send(bytes("DA:[100]", 'UTF-8'))

while(True):
	data = sock.recv(1000)
	print(datetime.datetime.now())
	decoded = base64.b64decode(data)
	print(decoded)

