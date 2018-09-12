import serial

ser = serial.Serial('/dev/ttyACM0',115200);
def readEncoders():
	ser.write("e 0\r")
	encoders = [None] * 3
	encoders[0] = ser.readline()
	ser.reset_input_buffer()
	ser.write("e 1\r")
	encoders[1] = ser.readline()
	ser.reset_input_buffer()
	ser.write("e 2\r")
	encoders[2] = ser.readline()
	ser.reset_input_buffer()
	return encoders
#def readUltraSound():
	
#def readInfrared():
	


while True:
	command = raw_input("Enter Command")
	ser.write(command + '\r')
	ser.reset_input_buffer()
	print ser.readline()
	
 




