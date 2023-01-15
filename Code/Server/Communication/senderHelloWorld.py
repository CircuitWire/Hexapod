import socket

#ip = "192.168.1.2"
ip = "10.1.1.1"
port = 60641
msg = b"hello"

print(f'Sending {msg} to {ip}:{port}')

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

sock.sendto(msg, (ip, port))


#if msg is buzzer
#cmd_buzzer
