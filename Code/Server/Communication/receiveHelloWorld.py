import socket

#ip = "192.168.1.2"
ip = "10.1.1.1"
port = 60641

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

sock.bind((ip, port))

print(f'Start listening to {ip}:{port}')


while True:
	data, addr = sock.recvfrom(1024)
	print(f"received message: {data}")
	#================== checking message request =====================
	print(str(data))
	
		
		

	#================== sending response back =====================
	
	#delay maybe 1-2 seconds 	
	
	#David's port
	#sock.sendto(b"ok", ("192.168.1.2", 60641))
