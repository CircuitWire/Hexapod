import socket
from Led import *
import myCode
import struct

ip = "192.168.1.6" 	#router
#ip = "10.1.1.1" 	
#ip = "10.85.51.50" #school
port = 60641

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

sock.bind((ip, port))

print(f'Start listening to {ip}:{port}')

led=Led()

while True:
	data, addr = sock.recvfrom(1024)
	print(f"received message: {data}")

	#================== sending response back =====================
	
	#delay maybe 1-2 seconds 	
	
	#David's port
	sock.sendto(b"ok from hex", ("192.168.1.2", 9016))
	

	#message = "ok from hex"
	#sock.sendto(message.encode(), ("192.168.1.3", 8889))
	

	#Ignore
	#================== sending response back =====================
	#David's port
	#sock.sendto(b"ok from hex", ("192.168.1.5", 49876))

	#print("message sent to David")
	#===============================================================

	#================== turn off ===============================
	if(str(data) == "b'Off'"):
		led.colorWipe(led.strip, Color(255, 0, 0))
		try:
			#Red wipe			
			led.colorWipe(led.strip, Color(0, 0, 0))   #turn off the light
			time.sleep(1)
		except KeyboardInterrupt:
			led.colorWipe(led.strip, Color(0, 0, 0))   #turn off the light
	#================== turn on ===============================
	if(str(data) == "b'Led'"):
		print("message is Led request")
		led.colorWipe(led.strip, Color(255, 255, 0))
		try:
			#Red wipe
			print ("\nRed wipe")
			led.colorWipe(led.strip, Color(20, 1, 20)) 
			time.sleep(1)
		except KeyboardInterrupt:
			led.colorWipe(led.strip, Color(0, 0, 0))   #turn off the light
			print ("\nEnd of program")
	#===================== forward =============================
	if(str(data) == "b'fwd'"):
		print("message is Forward")
		try:
			myCode.move(1, 2, 0, 12, 10, 0)
			myCode.relax()
		except KeyboardInterrupt:
			myCode.relax()
			print ("\nEnd of program")
	#===================== forward =============================
	if(str(data) == "b'relax'"):
		try:
			myCode.relax()
			sock.sendto(b"Relaxed", ("192.168.1.2", 9016))
		except KeyboardInterrupt:
			myCode.relax()
			print ("\nEnd of program")    
	#===================== forward =============================
	if(str(data) == "b'lidar'"):
		try:
			print(myCode.listenArduinoLiDar())
			distance = myCode.listenArduinoLiDar()
			distanceBytes = struct.pack('f', distance)
			print("$$$", distanceBytes)
			sock.sendto(distanceBytes, ("192.168.1.2", 9016))
			
			#print("hi")
			#print(myCode.listenArduinoLiDar())
			#distance = str(myCode.listenArduinoLiDar())
			#sock.sendto(b""+str.encode(distance), ("192.168.1.2", 9016))
			
		except KeyboardInterrupt:
			myCode.relax()
			print ("\nEnd of program")    
    #==================== rotate ===============================
	#if (str(data) == 'cw'):
    #    mycode.rotateCW(180)
      
    #==================== relax ===============================                                      
	#if (str(data) == 'relax'):
     #   mycode.relax() 	
		
		





