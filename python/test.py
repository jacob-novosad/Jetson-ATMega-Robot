import libomni as robot  #Library tha handles all the serial commands to arduino AtMega
import time



# libomni.motors(0,0,0)  first parameter is pwm signal for motor 1, 2nd is for motor 2, 3rd for motor 3
# the pwm signal is a value from 0 - 255 that sets the duty cycle for the motor. If you use a negative 
# value it will reverse the direction of the motor and use that pwm signal. The motors need atleast a
# PWM signal of atleast 55-60 to get the motor started
robot.motors(0,0,0)

time.sleep(1)
#for x in range(6):
#	print("encoder: "+robot.encoder(1)) # reads encoder value for wheel 2 (remember 0 is motor 1 and 1 is motor 2
#	print("rpm: "+ robot.rpm(1))     # reads rpm value by taking the change in encoder values over time 
#	time.sleep(.2)
uori = input ("u or i ")
senNum = int(input(" sensor number please "))
sleepTime= float(input("sleep time "))



while True:
	if(uori == 'i'):
		print("Infrared : "+robot.infrared(senNum))
	else:
		print("Ultrasound  : "+robot.ultraSound(senNum))
	time.sleep(sleepTime)



#	print(robot.encoder(0))
#	print(robot.encoder(1))
#	print(robot.encoder(2))
#	#time.sleep(.2)
#	print("Ultrasound 0: "+robot.ultraSound(0))
	#time.sleep(.2)
#	print("Ultrasound 1: "+robot.ultraSound(1))
#	print("Ultrasound 2: "+robot.ultraSound(2))
#	print("Ultrasound 3: "+robot.ultraSound(3))
#	print("Ultrasound 4: "+robot.ultraSound(4))
#	print("Ultrasound 5: "+robot.ultraSound(5))
#	print("-----------------------")
#	print("Infrared   0: "+robot.infrared(0))
#	print("Infrared   1: "+robot.infrared(1))
#	print("Infrared   2: "+robot.infrared(2))
#	print("Infrared   3: "+robot.infrared(3))
#	print("Infrared   4: "+robot.infrared(4))
#	print("Infrared   5: "+robot.infrared(5))
#	print("-----------------------")	time.sleep(1)


