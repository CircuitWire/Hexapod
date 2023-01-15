from Ultrasonic import *
from Control import *

c=Control() 		#Creating object 'control' of 'Control' class.
ultrasonic=Ultrasonic() #Creating object 'ultrasonic' of 'Ultrasonic' class.               


try:
     while True:
	data=ultrasonic.getDistance()   #Get the value
	print ("Obstacle distance is "+str(data)+"CM")
	time.sleep(1)
	
	 
	#data=['CMD_MOVE', '1', '0', '25', '10', '0']

	#for i in range(3):
		#data=['CMD_MOVE', '3', '0', '2', '10', '0']  #testing out gait mode "3"
		#c.run(data)
		#c.relax(True)
	    
except KeyboardInterrupt:
	print("\nEnd of program")

#========================= direction algorithm from video =================
def directionAlgorithm():
    global bs	# equals 0
    alg_dir = 'stop'
    biggestSpace = [0,0]
    for a in range(int(len(distances_list)/2)-10):
        sum = 1
        for b in range(10):
            sum += int(distances_list[2*a+2*b]) * int(distances_list[2*a+2*b+1])
        if biggestSpace[1] < sum:
            biggestSpace[0] = a+5
            biggestSpace[1] = sum
            bs = a*2
    if (biggestSpace[0] > 60) or biggestSpace[0] < 20:
        alg_dir = 'left'
    elif biggestSpace[0] > 20 and biggestSpace[0] < 50:
        alg_dir = 'right'
    else:
        alg_dir = 'forward'
    return alg_dir

