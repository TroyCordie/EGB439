#!/usr/bin/env python3

# needs to be run sudo
import stat
import os
import sys
import time

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

	if 0:
		os.system('mount {} {}'.format(disk, mnt) );
		os.system('cp -r {}/EGB439 {}/EGB439'.format(mnt,os.path.expanduser('~')) )
		os.system('shutdown -h now')
	print('mount {} {}'.format(disk, mnt) );
	print('cp -r {}/EGB439 {}/EGB439'.format(mnt,os.path.expanduser('~')) )
	print('shutdown -h now')
	
