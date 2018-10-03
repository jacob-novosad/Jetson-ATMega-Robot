import serial
import math
import xbox
import time

ser = serial.Serial('/dev/ttyACM0',115200);
joy = xbox.Joystick()




def readEncoders():
	encoders =  [None] *3
	for x in range(3):
		ser.write(("e %d\r" % (x)).encode())
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

#motors(00,00,00)
#motors(100,100,100)
#ser.write("clear\r".encode())



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


def xyThetaToWheelV(x,y,theta):
	wheel0Velocity=0
	wheel1Velocity=0
	wheel2Velocity=0
	
	wheel0Velocity = (((3.4907*y) - (0.6632* theta))* ((.03*60) / (2*3.14)));
	wheel1Velocity = (((3.023*x) - (1.7453*y) - (0.6632* theta))* ((.03*60) / (2*3.14)));
	wheel2Velocity = (((-3.023 * x) - (1.7453 * y) - (0.6632 * theta))* ((.03*60) / (2*3.14)));
	
	print("Wheel0 velocity: " +str(wheel0Velocity))
	print("Wheel1 velocity: " +str(wheel1Velocity))
	print("Wheel2 velocity: " +str(wheel2Velocity))
	print("PWM value for wheel0: " + str(wheel0Velocity*425))
	#print(velocityToPWM(wheel0Velocity));
	print("PWM value for wheel1: " + str(wheel1Velocity*425));
	print("PWM value for wheel2: " + str(wheel2Velocity*425));
	motors(wheel0Velocity*425,wheel1Velocity*425,wheel2Velocity*425)
	#motors(100,100,100)
#xyThetaToWheelV(0,0,0)
#readEncoders()
while True:
	command = input("Enter Command")
	command = command+'\r'
	
	ser.write(command.encode())
	print (ser.readline().decode("ascii"))

#while True:
#	x = float(input("enter x: "))
#	y = float(input("enter y: "))
#	theta = float(input("enter theta: "))
#	xyThetaToWheelV(x,y,theta)

#theta = 0
#while True:
#	theta = 0
#	time.sleep(.01)
#	if(joy.B()):
#		joy.close()
#		motors(0,0,0)
#		quit()
#	(x,y) = joy.leftStick()
#	(x1,y1) = joy.rightStick()
#	print("x: "+str(y))
#	print("y: "+str(x1))
#	if(joy.rightTrigger() > 0):
#		theta = joy.rightTrigger()
#	if(joy.leftTrigger() > 0):
#		theta= -joy.leftTrigger()
#	xyThetaToWheelV(-y,-x1/2,theta)




