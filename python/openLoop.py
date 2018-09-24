import serial

ser = serial.Serial('/dev/ttyACM0',115200);



def readEncoders():
	encoders =  [None] *3
	for x in range(3):
		ser.write(("e %d\r" % (x)).encode())
		encoders[x] =  (ser.readline().decode("ascii"))
		print (encoders[x])
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
		print(ser.readline().decode("ascii"))


#motors(00,00,00)
#motors(100,100,100)
#ser.write("clear\r".encode())




def velocityToPWM(velocity):
	PWMvalue = 0
	PWMValue = 124220*velocity**4 - 238623 * velocity**3 + 171697* velocity**2 - 54490* velocity + 6470
	return PWMValue


def xyThetaToWheelV(x,y,theta):
	wheel0Velocity=0
	wheel1Velocity=0
	wheel2Velocity=0
	
	wheel0Velocity = (((3.4907*y) - (0.6632* theta))* ((.03*60) / (2*3.14)));
	wheel1Velocity = (((3.023*x) - (1.7453*y) - (0.6632* theta))* ((.03*60) / (2*3.14)));
	wheel2Velocity = (((-3.023 * x) - (1.7453 * y) - (0.6632 * theta))* ((.03*60) / (2*3.14)));
	

	print("PWM value for wheel0: " + str(velocityToPWM(wheel0Velocity)))
	#print(velocityToPWM(wheel0Velocity));
	print("PWM value for wheel1: " + str(velocityToPWM(wheel1Velocity)));
	print("PWM value for wheel2: " + str(velocityToPWM(wheel2Velocity)));

xyThetaToWheelV(.5,0,0)
#while True:
#	command = input("Enter Command")
#	command = command+'\r'
	
#	ser.write(command.encode())
#	print (ser.readline().decode("ascii"))




