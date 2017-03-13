#!/usr/bin/env python3

# do not run as sudo
import stat
import os
import sys
import time
import subprocess

disk = '/dev/disk/by-label/PPI-UPDATE'
mnt = '/media/usb'


while True:
	time.sleep(2)

	try:
		if not stat.S_ISBLK(os.stat(disk).st_mode):
			continue;
	except:
		continue;

	print('disk present')

	os.system('sudo mount {} {}'.format(disk, mnt) );
	subprocess.call('cp -r ' + mnt + '/EGB439/. ' + os.path.expanduser('~') + '/EGB439', shell = True)
	output = subprocess.check_output('cp -r /media/usb/EGB439/. ' + os.path.expanduser('~') + '/EGB439', shell = True)
	os.system('sudo shutdown -h now')
