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
	
	data = data.decode(encoding='utf-8')
	coordList = (str(data)).split(',')
	                            	  
	#this is to check if coordinate is decimal or integer                            	                                      
	if(coordList[0].replace('.','',1).isdigit() and coordList[1].replace('.','',1).isdigit()):
		print("Coordlist[0]: ", coordList[0])
		print("Coordlist[1]: ", coordList[1])

	#================== sending response back =====================
	
	#delay maybe 1-2 seconds 	
	
	#David's port
	#sock.sendto(b"ok", ("192.168.1.2", 60641))
