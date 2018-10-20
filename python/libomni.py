import serial
import math
import time

#sets up serial connection to arduino atMega
ser = serial.Serial('/dev/ttyACM0',115200, timeout=.2);

#wait for serial connection to be established
time.sleep(1)

#clear buffers just incase garbage inside
ser.reset_input_buffer()
ser.reset_output_buffer()



# This functions sends  pwm signals to the motor and reverses the direction if given negative
# Example motor(255,0,0) would turn motor 0 on all the away and 1,2 off
# motor(125,-200,-100) motor 0 would have a half duty cycle, motor 1 would move backwards at a pwm of 200 etc...
def motors(m1,m2,m3):
	motorValues = [m1,m2,m3]
	for x in range(3):
		ser.write(("m %d %d %d\r" % (x, abs(motorValues[x]), int(motorValues[x]>=0))).encode())


#read encoder value from motor number given
def encoder(encoderNum):
	ser.write(("e %d \r" % (encoderNum)).encode())
	#time.sleep(.1)
	encoderValue = (ser.readline().decode("ascii"))
	return encoderValue.rstrip()

def ultraSound(ultraSoundNum):
	ser.write(("u %d \r" % (ultraSoundNum)).encode())
	#time.sleep(.1)
	ultraSoundValue = (ser.readline().decode("ascii"))
	return ultraSoundValue.rstrip()

def infrared(infraredNum):
	ser.write(("i %d \r" % (infraredNum)).encode())
	#time.sleep(.1)
	infraredValue = (ser.readline().decode("ascii"))
	return infraredValue.rstrip()


while True:
	print(infrared(0))


motors(0,0,0)
