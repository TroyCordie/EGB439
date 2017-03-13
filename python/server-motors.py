#!usr/bin/python3

import time
import traceback
import socket
import sys
import threading

import penguinPi as ppi

"""
" Constant Variables
"""
IP_ADDRESS = '0.0.0.0'
PORT = 43900
CHUNK_SIZE = 128

FN_MOTOR_SPEEDS = 'setMotorSpeeds'
FN_MOTOR_TICKS = 'getMotorTicks'
FN_DISPLAY_VALUE = 'setDisplayValue'
FN_DISPLAY_MODE = 'setDisplayMode'

debug = False;

"""
" Helper functions
"""
def executeRequestedFunction(requestData, connection):
	if debug:
		print('Function request: "{0}"' .format(requestData), file=sys.stderr)

	# Splice and dice
	data = requestData.decode("utf-8").split(',')
	fn = data[0]

	if debug:
		print('data contains ')
		print(data)

	# Decide what function should be run, and attempt to run it with the arguments
	if fn == FN_MOTOR_SPEEDS:
		motorA = int(data[1])
		motorB = int(data[2])

		mA.set_power(motorA)
		mB.set_power(motorB)

	elif fn == FN_MOTOR_TICKS:
		if debug:
			print("got here")
		mAticks = mA.get_ticks()
		mBticks = mB.get_ticks()
		#s = str(mAticks) + ' ' + str(mBticks) + ' ' + ':'
		s = str(mAticks) + ' ' + str(mBticks) + ' ' + '\n'
		
		if debug:
			print(s)

		b = s.encode('utf-8')

		connection.sendall(b)
		# TODO args -> BrickPi 'getTicks' function

		# send ticks back over 'connection

	# update the display
	elif fn == FN_DISPLAY_VALUE:
		#print('display value = ' + data[1]);
		display.set_value(int(data[1]));

	elif fn == FN_DISPLAY_MODE:
		#print('display mode = ' + data[1]);
		display.set_mode(data[1][0]);

"""
" Heartbeat thread, pulse the green LED periodically
"""
def HeartBeat:

    led = LED(AD_LED_G)

    while True:
        led.set_state(1);
        led.set_count(1000);
        time.sleep(5);

"""
" Main exectution block
"""
# Create a TCP/IP socket and bind the socket to the port
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_address = (IP_ADDRESS, PORT)
print('Starting up on {0}, port {1}' .format(server_address[0], server_address[1]), file=sys.stderr)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(server_address)

mA = ppi.Motor(ppi.AD_MOTOR_A)
mB = ppi.Motor(ppi.AD_MOTOR_B)
display = ppi.Display(ppi.AD_DISPLAY_A)

#initialise serial, and retrieve initial values from the Atmega
ppi.init()
mA.get_all()
mB.get_all()

# Initialise loop variables
connection = None
client_address = None

# initialize the heartbeat thread
heartbeat_thread = threading.Thread(target=HeartBeat, daemon=True)
heartbeat_thread.start()

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
        print('socket error: ', msg.value);
		pass
    except:
        print('command parser failed: ', data);
        pass
	finally:
		# Clean up the connection if it exists
		if connection is not None:
			connection.close()
