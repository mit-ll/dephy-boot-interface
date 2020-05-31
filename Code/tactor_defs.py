# DISTRIBUTION STATEMENT A. Approved for public release. Distribution is unlimited.
#
# This material is based upon work supported by the Under Secretary of Defense for Research and Engineering under Air Force Contract No. FA8702-15-D-0001. Any opinions, findings, conclusions or recommendations expressed in this material are those of the author(s) and do not necessarily reflect the views of the Under Secretary of Defense for Research and Engineering.
#
# © 2020 Massachusetts Institute of Technology.
#
# The software/firmware is provided to you on an As-Is basis
#
# Delivered to the U.S. Government with Unlimited Rights, as defined in DFARS Part 252.227-7013 or 7014 (Feb 2014). Notwithstanding any copyright notice, U.S. Government rights in this work are defined by DFARS 252.227-7013 or DFARS 252.227-7014 as detailed above. Use of this work other than as specifically authorized by the U.S. Government may violate any copyrights that exist in this work.
#
# P. Stegall 2020



# using the library for DRV2605 from github.com/pimoroni/drv2605-python/blob/master/
# to install on the system sudo pip3 install drv2605
from drv2605 import *

from platform import uname

system_info = uname()

if 'raspberry' in system_info[1] :
	import RPi.GPIO as GPIO  # use with Pi
	print('GPIO loaded for : {}'.format(system_info[1]))
elif 'nano' in system_info[1] :
	import Jetson.GPIO as GPIO # use with Nano
	print('GPIO loaded for : {}'.format(system_info[1]))
else :
	print('\n!!!\nPlatform {} not recognized.  GPIO not loaded.\n!!!\n'.format(system_info[1]))

class Tactor :
	
	setup_complete = False  # this is a class variable so is shared by all versions since we only need to setup once.  
	drv2605 = DRV2605() 
	def __init__(self, pin = 15, mode = "BOARD") :
		# if you want things to be easy between the Nano and Pi you should use board otherwise it looks like the numbering system is different.	
		self.mode = GPIO.getmode()
		if self.mode == None :  # if it is not already set set it
			self.mode = GPIO.BOARD if mode == "BOARD" else GPIO.BCM
			GPIO.setmode(self.mode)
		
		
		
		# since all of these have the same i2c address communication can only be one way (to the driver) , but that also means this only needs to be called once.  You can potentially put them on different I2C busses if you need them to be different.
		if (not self.setup_complete) :
			self.drv2605.reset()

			self.drv2605.set_feedback_mode('ERM')
			self.drv2605.set_library('TS2200 B')

			self.drv2605.auto_calibrate() # testing the default for now.  Did more specifics when doing C

			self.drv2605.set_mode('Edge Trigger')

			self.drv2605.set_sequence(17)
			
			self.setup_complete = True
		
		self.pin = pin;
		GPIO.setup(self.pin,GPIO.OUT)
		
	def __del__ (self) :
		self.drv2605.stop()
		self.drv2605.reset()  # essential power cycles the driver so it returns to standby.  This way the tactors don't keep going when the trigger pin floats high.
		# TODO : add sleep state
			
	def go(self) :
		GPIO.output(self.pin,GPIO.HIGH)
		GPIO.output(self.pin,GPIO.LOW)