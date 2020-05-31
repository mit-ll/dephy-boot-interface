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

import os, sys
thisdir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(thisdir)

from pyFlexsea import *
from pyFlexsea_def import *
from fxUtil import *
from exo_defs import *
from sync_led_def import *
#from TorquePredictionModelTFLite import TorquePredictionModelTFLite
#from TorquePredictionPipeline import TorquePredictionPipeline

import time

# from flexseapython.flexsea_demo.readonly import fxReadOnly
# from flexseapython.flexsea_demo.opencontrol import fxOpenControl
# from flexseapython.flexsea_demo.currentcontrol import fxCurrentControl
# from flexseapython.flexsea_demo.positioncontrol import fxPositionControl
# from flexseapython.flexsea_demo.two_devices_positioncontrol import fxTwoDevicePositionControl
# from flexseapython.flexsea_demo.two_devices_leaderfollower import fxLeaderFollower
# from flexseapython.flexsea_demo.twopositioncontrol import fxTwoPositionControl
# from flexseapython.flexsea_demo.userRW import fxUserRW

#from readData import *
#from flexseapython.flexsea_demo.streamManager import Stream

ZEROING_CURRENT = 1000 # mA

def zero_boots (leftExo,rightExo) :
	
	# the gains need to be defined before this is called
	
	leftExo.set_controller(CTRL_CURRENT)
	rightExo.set_controller(CTRL_CURRENT)
	
	leftExo.set_exo_current(ZEROING_CURRENT)
	rightExo.set_exo_current(ZEROING_CURRENT)
	
	time.sleep(1) # wait a second
	
	
	leftExo.zero_encoders()
	rightExo.zero_encoders()
	
	print("zeroBoots : leftExo.ankle_ticks_offset = " + str(leftExo.ankle_ticks_offset) )
	print("zeroBoots : rightExo.ankle_ticks_offset = " + str(rightExo.ankle_ticks_offset) )
	
	
	leftExo.set_exo_current(0)
	rightExo.set_exo_current(0)
	leftExo.set_controller(CTRL_NONE)
	rightExo.set_controller(CTRL_NONE)
	
	
def zero_boot (exo) :
	
	# the gains need to be defined before this is called
	
	exo.set_controller(CTRL_CURRENT)
	
	
	exo.set_exo_current(ZEROING_CURRENT)
		
	time.sleep(2) # wait a second
	
	# not sure if this will update the original encoder offsets or if this is a new instance of the boots.
	exo.zero_encoders()
		
	print("zeroBoots : exo.ankle_ticks_offset = " + str(exo.ankle_ticks_offset) )
	
	exo.set_exo_current(0)
	exo.set_controller(CTRL_NONE)
	
def startStopCapture(sync_light, leftExo, rightExo) :
	print('starting sync pattern')
	# low for 1 period capture both ends
	sync_light.set_state(0)
	leftExo.read_data()
	rightExo.read_data()
	time.sleep(sync_light.period)
	leftExo.read_data()
	rightExo.read_data()
	time.sleep(.001)
	
	# high for 2 period capture both ends
	sync_light.set_state(1)
	leftExo.read_data()
	rightExo.read_data()
	time.sleep(2 * sync_light.period)
	leftExo.read_data()
	rightExo.read_data()
	time.sleep(.001)
	
	# low for 1 period capture both ends
	sync_light.set_state(0)
	leftExo.read_data()
	rightExo.read_data()
	time.sleep(sync_light.period)
	leftExo.read_data()
	rightExo.read_data()


def main():
	
	sync_led = SyncLed(period = 1)
	
	
	scriptPath = os.path.dirname(os.path.abspath(__file__))
	fpath = scriptPath + '/com.txt'
	ports, baudRate = loadPortsFromFile(fpath)
	print('Loaded ports: ' + str(ports))
	print('Baud Rate: ' + str(baudRate))	
	# must turn on left boot then right boot.  Otherwise this will break.
	# TODO: find a better solution for this but currently we don't have access to boot specific IDs.
	idx = 0
	print('creating left boot')
	left_boot = ExoBoot (LEFT, ports[0], int(baudRate), idx,shouldExoLog = False, shouldLog = True, frequency = 1000, use_tactor = False) # recent change the first line is now the baud rate
	idx +=1
	right_boot = ExoBoot (RIGHT, ports[1], int(baudRate), idx, shouldExoLog = False, shouldLog = True, frequency = 1000, use_tactor = False)
	
	if False : # ( (not left_boot.id) or (not right_boot.id) ): 
		print("At least one boot is missing")
	else : 	
		# TODO: Add in gains for current and position
		left_boot.define_current_gains(100,32)
		right_boot.define_current_gains(100,32)
		
		zero_boots(left_boot, right_boot);
		#zero_boot(left_boot);
		#print("main : left_boot.ankle_ticks_offset = " + str(left_boot.ankle_ticks_offset ) )
		
		user_mass = 100
		rspg = 0
		opg = 27.1
		ppg = 52.4
		spg = 62.7
		ot = 2
		npt = .175
		
		tactor_percent = 5
		
		
		left_boot.tactor_trigger_percent = tactor_percent	# set the percent of gait the tactor should trigger
		right_boot.tactor_trigger_percent = tactor_percent	# set the percent of gait the tactor should trigger
		
		left_boot.init_collins_profile(mass = user_mass, ramp_start_percent_gait = rspg, onset_percent_gait = opg, peak_percent_gait = ppg, stop_percent_gait = spg, onset_torque = ot, normalized_peak_torque = npt)	# initialize the Zhang/Collins profile
		right_boot.init_collins_profile(mass = user_mass, ramp_start_percent_gait = rspg, onset_percent_gait = opg, peak_percent_gait = ppg, stop_percent_gait = spg, onset_torque = ot, normalized_peak_torque = npt)	# initialize the Zhang/Collins profile
		
		left_boot.set_controller(CTRL_CURRENT)
		right_boot.set_controller(CTRL_CURRENT)

		start_time = 0
		
		startStopCapture(sync_led, left_boot, right_boot)
		
		print('starting loop')
		try:
			while True:
				# put main code loop here it will stop with a CTRL+c
				# while ((time.monotonic()-start_time) <= (1/left_boot.frequency)) : # if you run boot interface loop faster than the dephy frequency the cue will fill up and it will take a long time shut down and to have the data come in.  This just waits till the boot period has passed then runs.
					# pass
				# start_time = time.monotonic()
				sync_led.check()
				
				left_boot.read_data()
				right_boot.read_data()
				
				left_boot.run_collins_profile(external_read = True)	# apply the torque appropriate to the Zhang/Collins profile
				right_boot.run_collins_profile(external_read = True)	# apply the torque appropriate to the Zhang/Collins profile
				
				time.sleep(1/left_boot.frequency)  # Lazy alternative to the waiting while loop but will run a bit slower.
				
				
		except KeyboardInterrupt:
			print("KeyboardInterrupt has been caught.")

	
	left_boot.set_exo_current(0)
	right_boot.set_exo_current(0)
	
	time.sleep(.03)
	
	left_boot.set_controller(CTRL_NONE)
	right_boot.set_controller(CTRL_NONE)
	
	
	startStopCapture(sync_led, left_boot, right_boot)
	
	del left_boot
	del right_boot
	
	cleanupPlanStack()
	
	

main()
