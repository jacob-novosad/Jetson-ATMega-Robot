import serial
import math
import xbox
import time

ser = serial.Serial('/dev/ttyACM0',115200, timeout=.1);

while(ser.isOpen() == False):
	time.sleep(.001)

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
	wheel0Velocity=0
	wheel1Velocity=0
	wheel2Velocity=0

	wheel0Velocity = ((318.3099*yd) - (60.4789* thetad));
	wheel1Velocity = ((275.6644*xd) - (159.1549*yd) - (60.4789* thetad));
	wheel2Velocity = ((-275.6644 * xd) - (159.1549 * yd) - (60.4789 * thetad));

	print("Wheel0 RPM: " +str(wheel0Velocity))
	print("Wheel1 RPM: " +str(wheel1Velocity))
	print("Wheel2 RPM: " +str(wheel2Velocity))
	wheel0Velocity *= 10
	wheel1Velocity *= 10
	wheel2Velocity *= 10
	

	velocityValues(int(wheel0Velocity),int(wheel1Velocity),int(wheel2Velocity))
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
