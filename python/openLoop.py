import serial
import math
import xbox
import time
import numpy as np

ser = serial.Serial('/dev/ttyACM0',115200, timeout=.1);

time.sleep(1)

ser.reset_input_buffer()
ser.reset_output_buffer()
#joy = xbox.Joystick()




def readEncoders():
	encoders =  [None] *3
	for x in range(3):
		ser.write( ("e %d\r" % (x) ).encode())
		encoders[x] =  (ser.readline().decode("ascii"))
		print(encoders[x])
	return encoders

#def readUltraSound():

#def readInfrared():

# Fun function that takes all 3 motor PWM value from -255  to 255 and interprets it 
# into the correct command to send to arduino. Remember with Python3  Pyserial pretty much
# expects everything  in forms of bytes so we encode it into a byte and decode the output from
# atmega from bytes to ascii string
def motors(m1,m2,m3):
	motorValues = [m1,m2,m3]
	for x in range(3):
		ser.write(("m %d %d %d\r" % (x, abs(motorValues[x]), int(motorValues[x]>=0))).encode())
		#print(ser.readline().decode("ascii"))
	#readEncoders()
def velocityValues(m1,m2,m3):
	motorValues = [m1,m2,m3]
	print(motorValues)
	ser.write(('v %d %d %d \r'  % (motorValues[0],motorValues[1],motorValues[2])).encode())
	
	#for x in range(3):
	#	time.sleep(.5)
	#	ser.write(('v %d %d \n \r' % (x,motorValues[x])).encode()) #turning into int should fix !!!!!!!!!!!!!!!!!!
	print("Finished all commands")


theta = 0
def velocityToPWM(velocity):
	theta = 0
	isNegative = False
	if(velocity < 0):
		velocity = abs(velocity)
		isNegative = True
	if(velocity == 0):
		return 0
	if(velocity > 0 and velocity < .36):
		if(isNegative):
			return -58
		return 58
	if(velocity >= .656):
		if(isNegative):
			return -252
		return 252

	PWMvalue = 0
	PWMValue = 124220*velocity**4 - 238623 * velocity**3 + 171697* velocity**2 - 54490* velocity + 6470
	if(isNegative):
		return PWMValue*-1
	return PWMValue


def xyThetaToWheelV(xd,yd,thetad):

	r = 0.03 # radius of each wheel [m]
	l = 0.19 # distance from each wheel to the point of reference [m]
 
	xd_des = xd # velocity in the x-direction in the local frame [m/s]
	yd_des = yd # velocity in the y-direction in the local frame [m/s]
	thd_des = thetad # velocity in the x-direction in the local frame [rad/sa]
 
	vel_des = np.array([xd_des,yd_des,thd_des])[:,None]
 
	FK_M = (2*np.pi*r/60)*np.array([1/np.sqrt(3),0,-1/np.sqrt(3),-1/3,2/3,-1/3,-1/(3*l),-1/(3*l),-1/(3*l)]).reshape(3,3) # Forward kinematics matrix
 
	IK_M = np.linalg.inv(FK_M) # Inverse kinematics matrix
 
	motor_spd_vec = np.dot(IK_M,vel_des)
 
	wheel1RPM = motor_spd_vec[0] # motor 2 speed [rpm]
	wheel0RPM = motor_spd_vec[1] # motor 1 speed [rpm]
	wheel2RPM = motor_spd_vec[2] # motor 3 speed [rpm]


	print("Wheel0 RPM: " +str(wheel0RPM))
	print("Wheel1 RPM: " +str(wheel1RPM))
	print("Wheel2 RPM: " +str(wheel2RPM))
	wheel0RPM *= 10
	wheel1RPM *= 10
	wheel2RPM *= 10
	

	velocityValues(int(wheel0RPM),int(wheel1RPM),int(wheel2RPM))
	#motors(100,100,100)

#velocityValues(0,0,0)
#xyThetaToWheelV(0,0,0)
#readEncoders()
readEncoders()

mode = str(input("Enter mode. s for serial, t for input tester, c for controller, g for graph mode "))

if(mode == 's'):

############## Simple Serial Communicator to Arduino ##############
	while True:
		command = input("Enter Command")
		command = command+'\r'
		ser.write(command.encode())
		print (ser.readline().decode("ascii"))
		
elif mode == 't':
################ Simple Input Tester Loop ###############
	while True:
		yesNo = input("do you want to quit y/n")
		if(yesNo == 'y'):
			xyThetaToWheelV(0,0,0)
			ser.close()
			quit()
		time.sleep(.5)
		x = float(input("enter x: "))
		y = float(input("enter y: "))
		theta = float(input("enter theta: "))
		mytime  = float(input("enter time to run: "))
		xyThetaToWheelV(x,y,theta)
		time.sleep(mytime)
		xyThetaToWheelV(0,0,0)

#velocityValues(1800,1800,1800)

############### Contoller demo for testing  ################

elif mode == 'c':
#	joy = xbox.Joystick()
	theta = 0
	while True:
		theta = 0
		time.sleep(.3)
		if(joy.B()):
			joy.close()
			motors(0,0,0)
			quit()
		(x,y) = joy.leftStick()
		(x1,y1) = joy.rightStick()
		print("x: "+str(y))
		print("y: "+str(x1))
		if(joy.rightTrigger() > 0):
			theta = joy.rightTrigger()
		if(joy.leftTrigger() > 0):
			theta= -joy.leftTrigger()
		xyThetaToWheelV(-y/1.5,-x1/1.5,theta*2)


###################### GRAPH  ##############################

elif mode == 'g':
	i=0
	file = open("sampleText.txt","w")
#	joy = xbox.JoyStick()
	while True:
		command = input("Enter Command")
		command = command+'\r'
		ser.write(command.encode())

		while True:
			if(ser.inWaiting() > 3):
				myString =ser.readline().decode("ascii")
				file.write(myString)
			if(joy.B()):
				print('quitting')
				velocityValues(0,0,0)
				break
#readline check to make sure serial to read
