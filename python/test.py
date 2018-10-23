import libomni as robot  #Library tha handles all the serial commands to arduino AtMega
import time



# libomni.motors(0,0,0)  first parameter is pwm signal for motor 1, 2nd is for motor 2, 3rd for motor 3
# the pwm signal is a value from 0 - 255 that sets the duty cycle for the motor. If you use a negative 
# value it will reverse the direction of the motor and use that pwm signal. The motors need atleast a
# PWM signal of atleast 55-60 to get the motor starte


#time.sleep(1)


robot.enablePID(1)
#time.sleep(1)
#for x in range(6):
#	print("encoder: "+robot.encoder(1)) # reads encoder value for wheel 2 (remember 0 is motor 1 and 1 is motor 2
#	print("rpm: "+ robot.rpm(1))     # reads rpm value by taking the change in encoder values over time 
#	time.sleep(.2)
#uori = input ("u or i ")
#senNum = int(input(" sensor number please "))
#sleepTime= float(input("sleep time "))
while True:
	print("0 : "+str(robot.infrared(0)))
	print("1    : "+str(robot.infrared(1)))
notSeen = False
seen    = False
while True:
	ultra = robot.infrared(2)
	if(float(ultra) >= 30 and float(robot.infrared(1)) >=30):
		if(notSeen == False):
			robot.moveXYTheta(.3,0,0)
			notSeen = True
		seen=False
	else:
		robot.moveXYTheta(0,0,2)
		time.sleep(.65)
		notSeen = False
	
