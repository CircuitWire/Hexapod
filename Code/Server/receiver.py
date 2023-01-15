import socket
from Led import *
from myCodeClass import *

#ip = "192.168.1.2" 	#router
ip = "192.168.1.131" 	#verizon
port = 49876

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

sock.bind((ip, port))

print(f'Start listening to {ip}:{port}')

led=Led()
mycode = myCodeClass()

while True:
	data, addr = sock.recvfrom(1024)
	print(f"received message: {data}")

	#================== sending response back =====================
	
	#delay maybe 1-2 seconds 	
	
	#David's port
	sock.sendto(b"ok from hex", ("192.168.1.3", 8889))
	

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
		led.colorWipe(led.strip, Color(255, 0, 0))
		try:
			#Red wipe
			print ("\nRed wipe")
			led.colorWipe(led.strip, Color(20, 1, 20)) 
			time.sleep(1)
		except KeyboardInterrupt:
			led.colorWipe(led.strip, Color(0, 0, 0))   #turn off the light
			print ("\nEnd of program")
	#===================== forward =============================
    if (str(data) == 'fwd'):
      mycode.forward()  
    
    #==================== rotate ===============================
	if (str(data) == 'cw'):
      mycode.rotateCW(180)
      
    #==================== relax ===============================                                      
	if (str(data) == 'relax'):
      mycode.relax() 	
		
		





