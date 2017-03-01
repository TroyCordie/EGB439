#!usr/bin/python3

import time
import traceback
import socket
import sys

import penguinPi as ppi

"""
" Constant Variables
"""
IP_ADDRESS = '0.0.0.0'
PORT = 43900
CHUNK_SIZE = 128

FN_MOTOR_SPEEDS = 'setMotorSpeeds'
FN_MOTOR_TICKS = 'getMotorTicks'

"""
" Helper functions
"""
def executeRequestedFunction(requestData, connection):
	print('Function request: "{0}"' .format(requestData), file=sys.stderr) #>>sys.stderr, 'Starting up on %s, port %s' % server_address

	# Splice and dice
	data = requestData.decode("utf-8").split(',')
	fn = data[0]
	print('data contains ')
	print(data)

	# Decide what function should be run, and attempt to run it with the arguments
	if fn == FN_MOTOR_SPEEDS:
		motorA = int(data[1])
		motorB = int(data[2])

		mA.set_power(motorA)
		mB.set_power(motorB)


	elif fn == FN_MOTOR_TICKS:
		print("got here")
		mAticks = mA.get_ticks()
		mBticks = mB.get_ticks()
		s = str(mAticks) + ' ' + str(mBticks) + ' ' + ':'
		print(s)

		b = s.encode('utf-8')

		connection.sendall(b)
		# TODO args -> BrickPi 'getTicks' function

		# send ticks back over 'connection


"""
" Main exectution block
"""
# Create a TCP/IP socket and bind the socket to the port
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_address = (IP_ADDRESS, PORT)
print('Starting up on {0}, port {1}' .format(server_address[0], server_address[1]), file=sys.stderr)
sock.bind(server_address)

mA = ppi.Motor(ppi.AD_MOTOR_A)
mB = ppi.Motor(ppi.AD_MOTOR_B)

#initialise serial, and retrieve initial values from the Atmega
ppi.init()
mA.get_all()
mB.get_all()

# Initialise loop variables
connection = None
client_address = None

# Start listening for incoming commands
sock.setblocking(0)
sock.listen(1)
while True:
	# Check if any new commands have been received
	try: # try catch used because error thrown when there's connection - i.e. no signal transmitted
		connection, client_address = sock.accept()
		recv_str = connection.recv(CHUNK_SIZE)
		data = recv_str
		while not recv_str:
			recv_str = connection.recv(CHUNK_SIZE)
			data += recv_str
		# TODO should check if all data was received.... but too lazy

		# Send the data to the function processor
		executeRequestedFunction(data.rstrip(), connection)
	except socket.error as msg:
		pass
	finally:
		# Clean up the connection if it exists
		if connection is not None:
			connection.close()