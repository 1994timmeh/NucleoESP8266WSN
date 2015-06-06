import socket
import sys
import base64
import datetime
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

server_addr = ('192.168.3.1', 8888)
sock.connect(server_addr)
sock.send("DA:[100]")

while(True):
	data = sock.recv(1000)
	print(datetime.datetime.now())
	decoded = base64.b64decode(data)
	dec_hex = ":".join(hex(ord(x)).zfill(4) for x in decoded)
	print(dec_hex)
	print('\a')
