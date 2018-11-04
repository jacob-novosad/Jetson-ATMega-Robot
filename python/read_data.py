import serial
import math
import xbox
import time
import numpy as np
import openLoop


if mode == 'g':
	i=0
	file = open("sampleText.txt","w")

	while True:
		command = input("Enter Command")
		command = command+'\r'
		ser.write(command.encode())

		while True:
			if(ser.inWaiting() > 3):
				myString =ser.readline().decode("ascii")
				file.write(myString)
	
