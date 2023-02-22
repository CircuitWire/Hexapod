import serial
import RPi.GPIO as GPIO
import time

ser=serial.Serial("/dev/ttyACM0",115200)  #change ACM number as found from ls /dev/tty/ACM*
ser.baudrate=115200
def blink(pin):
    GPIO.output(pin,GPIO.HIGH)  
    time.sleep(1)  
    GPIO.output(pin,GPIO.LOW)  
    time.sleep(1)  
    return

GPIO.setmode(GPIO.BOARD)
GPIO.setup(11, GPIO.OUT)
while True:
    read_ser=ser.readline()
    print(read_ser)
    distance = str(read_ser).split("'")
    distance_Inches = str(distance[1]).split("\x5c")
    print(str(distance_Inches[0]))
    if(read_ser=="Hello From Arduino!"):
        blink(11)
