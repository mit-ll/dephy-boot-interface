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
	
import time

class SyncLed: 
	def __init__ (self, pin = 40, period = 1, mode = "BOARD") :
		# if you want things to be easy between the Nano and Pi you should use board otherwise it looks like the numbering system is different.
		self.mode = GPIO.getmode()
		if self.mode == None :  # if it is not already set set it
			self.mode = GPIO.BOARD if mode == "BOARD" else GPIO.BCM
			GPIO.setmode(self.mode)
		self.pin = pin
		self.period = period
		self.state = 0
		self.start_time = time.monotonic()
		
		GPIO.setup(self.pin, GPIO.OUT)
	
	def __del__ (self) :
		GPIO.cleanup()  # if cleanup is called the pin will go high at the end
		
	
	def get_state(self) :
		return (GPIO.input(self.pin))
	
	def set_state(self, state) :
		self.state = state
		if (self.state) : # low is on for the LED
			GPIO.output(self.pin,GPIO.HIGH)
			#print("SyncLed :: set_state(state) : LED on")
		else :
			GPIO.output(self.pin,GPIO.LOW)
			#print("SyncLed :: set_state(state) : LED off")
	
	def check(self) :
		cur_time = time.monotonic()
		should_change = (self.period / 2) < (cur_time - self.start_time)
		#print("pin state : " + str(self.get_state()))
		if  should_change :
			self.set_state(not self.get_state() )
			self.start_time = cur_time
			
		return should_change
			
	
