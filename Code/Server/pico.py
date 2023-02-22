from machine import pin
from time import sleep
led=Pin(25,Pin.OUT)

while True:
    led.toggle()
    sleep(0.5)