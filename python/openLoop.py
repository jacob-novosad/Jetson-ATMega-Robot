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


motors(00,00,00)
#motors(100,100,100)
#ser.write("clear\r".encode())
while True:
	command = input("Enter Command")
	command = command+'\r'
	
	ser.write(command.encode())
	print (ser.readline().decode("ascii"))




