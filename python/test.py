import omrebot
from omrebot import OmreBot


robot = OmreBot('/dev/ttyACM0')


robot.port = "ha"

mode = str(input("Enter mode. s for serial, t for input tester, c for controller, g for graph mode "))

if(mode == 's'):

############## Simple Serial Communicator to Arduino ##############
	while True:
		command = input("Enter Command")
		command = command+'\r'
		robot.ser.write(command.encode())
		print (ser.readline().decode("ascii"))

