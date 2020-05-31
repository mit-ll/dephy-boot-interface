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


# !!!
# Make sure to change the port to match what is in the tcp_server_gui.py or what is controlling the system.  Also change the ip address to the computer that is handling the controls.
# !!!

import os, sys
thisdir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(thisdir)

from pyFlexsea import *
from pyFlexsea_def import *
from fxUtil import *
from exo_defs import *
from sync_led_def import *


import time
from tcp_handler_def import *

from study_one import *
from study_zero import *




HOST = <IP_ADDRESS_OF_THE_HOST>  # example keep the quotes :  '192.168.1.1'
PORT = <PORT_THE_HOST_IS_LISTENING_TO> # example: 67876

STUDY_IDX = 0
TRIAL_IDX = 1
MSG_IDX = 2
USER_MASS_IDX = MSG_IDX + 1


PROGRAM_EXIT_MSG = 255
READY_FOR_TRIAL_MSG = 254
STOP_CURRENT_TRIAL_MSG = 254
CHECK_IN_MSG = 253
SYNC_FINISHED_END_MSG = 252
TRIAL_EXIT_MSG = 251
TRIAL_BEGIN_MSG = 250
SYNC_FINISHED_BEGINNING_MSG = 249
TREADMILL_STOP_MSG = 252





ZEROING_CURRENT = 1000 # mA

def zero_boots (leftExo,rightExo) :
	# TODO: Change this so it is speed controlled till a current limit is hit.
	# the gains need to be defined before this is called
	leftExo.set_controller(CTRL_CURRENT)
	rightExo.set_controller(CTRL_CURRENT)
	
	leftExo.set_exo_current(ZEROING_CURRENT)
	rightExo.set_exo_current(ZEROING_CURRENT)
	
	time.sleep(1) # wait a second
	
	# not sure if this will update the original encoder offsets or if this is a new instance of the boots.
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
	# low for 1 s capture both ends
	sync_light.set_state(0)
	leftExo.read_data()
	rightExo.read_data()
	time.sleep(sync_light.period)
	leftExo.read_data()
	rightExo.read_data()
	time.sleep(.001)
	
	# high for 2 s capture both ends
	sync_light.set_state(1)
	leftExo.read_data()
	rightExo.read_data()
	time.sleep(2 * sync_light.period)
	leftExo.read_data()
	rightExo.read_data()
	time.sleep(.001)
	
	# low for 1 s capture both ends
	sync_light.set_state(0)
	leftExo.read_data()
	rightExo.read_data()
	time.sleep(sync_light.period)
	leftExo.read_data()
	rightExo.read_data()

def trial_simple_start (sync_light, leftExo, rightExo,  com, study_num, trial_num):
	# restart trial timer
	# start new recording file
	leftExo.data_file = leftExo.log_init()
	rightExo.data_file = rightExo.log_init()
	
	startStopCapture(sync_light, leftExo, rightExo)
	com.send_recv(send_msg = [study_num, trial_num, SYNC_FINISHED_BEGINNING_MSG])
	time.sleep(.010) # pause for a bit to let messages pass
	com.send_recv(send_msg = [study_num,  trial_num, TRIAL_BEGIN_MSG])
	com.send_msg = [study_num,  trial_num, CHECK_IN_MSG]
	
def trial_simple_end (sync_light, leftExo, rightExo, com, study_num, trial_num):
	com.send_recv(send_msg = [study_num, trial_num,  TRIAL_EXIT_MSG]);
	com.send_msg = [study_num, trial_num, CHECK_IN_MSG]
	# wait for the treadmill to stop if program exit received go strait to exit
	while com.recv_msg[MSG_IDX] != TREADMILL_STOP_MSG and (com.recv_msg[MSG_IDX] != PROGRAM_EXIT_MSG):
		com.heartbeat()
		sync_light.check()
	
	startStopCapture(sync_light, leftExo, rightExo)
	com.send_recv(send_msg = [study_num, trial_num,  SYNC_FINISHED_END_MSG]);
	sync_light.set_state(1)
	
	com.send_msg = [0, 0,  READY_FOR_TRIAL_MSG]; # this just preps for the next

def main():
	
	# socket_host = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	# socket_send_msg = bytearray([0, 254])
	# socket_recv_msg = None
	
	coms = TcpHandler(port = PORT, host = HOST)
	# coms.send_recv()

	
	sync_led = SyncLed(period = 2)
	
	
	scriptPath = os.path.dirname(os.path.abspath(__file__))
	fpath = scriptPath + '/com.txt'
	ports, baudRate = loadPortsFromFile(fpath)
	print('Loaded ports: ' + str(ports))
	print('Baud Rate: ' + str(baudRate))	
	# must turn on left boot then right boot.  Otherwise this will break.
	# TODO: find a better solution for this but currently we don't have access to boot specific IDs.
	idx = 0
	print('creating left boot')
	left_boot = ExoBoot (LEFT, ports[0], int(baudRate), idx,shouldExoLog = False, shouldLog = False, frequency = 1000) # recent change the first line is now the baud rate
	idx +=1
	right_boot = ExoBoot (RIGHT, ports[1], int(baudRate), idx, shouldExoLog = False, shouldLog = False, frequency = 1000)
	
	if False : # ( (not left_boot.id) or (not right_boot.id) ): 
		print("At least one boot is missing")
	else : 	
		# TODO: Add in gains for current and position
		left_boot.define_current_gains(100,32)
		right_boot.define_current_gains(100,32)
		left_boot.define_position_gains(10,0)
		right_boot.define_position_gains(10,0)
		
		zero_boots(left_boot, right_boot);
		left_boot.should_log = True
		right_boot.should_log = True
		#zero_boot(left_boot);
		#print("main : left_boot.ankle_ticks_offset = " + str(left_boot.ankle_ticks_offset ) )
		
		
		left_boot.set_controller(CTRL_CURRENT)
		right_boot.set_controller(CTRL_CURRENT)
		
#		model = TorquePredictionModelTFLite(offline_testing=False)
#		torque_pred_pipeline = TorquePredictionPipeline(model)
		start_time = 0
		
		#startStopCapture(sync_led, left_boot, right_boot)
		
		
		
		# left_boot.init_collins_profile(mass = 100, ramp_start_percent_gait = 0, onset_percent_gait = 27.1, peak_percent_gait = 52.4, stop_percent_gait = 62.7, onset_torque = 2, normalized_peak_torque = .25)
		# right_boot.init_collins_profile(mass = 100, ramp_start_percent_gait = 0, onset_percent_gait = 27.1, peak_percent_gait = 52.4, stop_percent_gait = 62.7, onset_torque = 2, normalized_peak_torque = .25)
		
		# left_boot.tactor_trigger_percent = 25
		# right_boot.tactor_trigger_percent = 25
		
		coms.send_recv(send_msg = [0, 0, READY_FOR_TRIAL_MSG])
		sync_led.set_state(1)
		
		user_mass = 0
		
		try:
			while coms.recv_msg[MSG_IDX] != PROGRAM_EXIT_MSG :
				# put main code loop here it will stop with a CTRL+c
				# while ((time.monotonic()-start_time) <= (1/left_boot.frequency)) : # if you run this loop faster than the dephy frequency the cue will fill up and it will take a long time shut down and to have the data come in.
					# a=0
				# start_time = time.monotonic()
				
				coms.heartbeat()
				
				if coms.recv_msg[USER_MASS_IDX] != user_mass :
					user_mass = coms.recv_msg[USER_MASS_IDX]
				
				
				# Study 0 ------------------------------------------------------------------------------------
				if coms.recv_msg[STUDY_IDX] == 0 :
					coms.send_msg[STUDY_IDX] = 0
					if coms.recv_msg[TRIAL_IDX] == 0 and coms.recv_msg[MSG_IDX] == TRIAL_BEGIN_MSG:
						try :
							if user_mass != -1 :
								s0 = STUDY0( user_mass, left_boot, right_boot)
								# coms.send_msg[MSG_IDX] = TRIAL_EXIT_MSG
								coms.send_recv(send_msg = [0, 0,  TRIAL_EXIT_MSG]);
								coms.send_msg = [0, 0,  READY_FOR_TRIAL_MSG]; # this just preps for the next heartbeat
								time.sleep(3) # this is just to give me a chance to change values with packetsender
							else : 
								print('Study 0 initialization called but user mass not set')
						except IndexError :
							print('Study 0 initialization index error')
					elif coms.recv_msg[TRIAL_IDX] == 1 and coms.recv_msg[MSG_IDX] == TRIAL_BEGIN_MSG:
						
						print('Trial 1')
						trial_simple_start (sync_led, left_boot, right_boot,  coms, 0, 1)
						trial_running = s0.trial1(restart_trial = True)
						while ( trial_running and (coms.recv_msg[MSG_IDX] != STOP_CURRENT_TRIAL_MSG) and (coms.recv_msg[MSG_IDX] != PROGRAM_EXIT_MSG)) :
							#print('S0T1 : in Loop')
							time.sleep(1/(2*left_boot.frequency))
							sync_led.check()
							coms.heartbeat()
							trial_running = s0.trial1()
						
						trial_simple_end (sync_led, left_boot, right_boot,  coms, 0, 1)
					
					
					
					
					elif coms.recv_msg[TRIAL_IDX] == 2 and coms.recv_msg[MSG_IDX] == TRIAL_BEGIN_MSG:
						sync_led.set_state(not sync_led.get_state() )
						# coms.send_msg[MSG_IDX] = 251
						coms.send_recv(send_msg = [0, 0,  TRIAL_EXIT_MSG]);
						coms.send_msg = [0, 0,  READY_FOR_TRIAL_MSG]; # this just preps for the next heartbeat
					


					
					elif coms.recv_msg[TRIAL_IDX] == 3 and coms.recv_msg[MSG_IDX] == TRIAL_BEGIN_MSG:
						zero_boots(left_boot,right_boot)
						# coms.send_msg[MSG_IDX] = 251
						coms.send_recv(send_msg = [0, 0,  TRIAL_EXIT_MSG]);
						coms.send_msg = [0, 0,  READY_FOR_TRIAL_MSG]; # this just preps for the next heartbeat

					elif coms.recv_msg[TRIAL_IDX] == 4 and coms.recv_msg[MSG_IDX] == TRIAL_BEGIN_MSG:
						zero_boot(left_boot)
						# coms.send_msg[MSG_IDX] = 251
						coms.send_recv(send_msg = [0, 0,  TRIAL_EXIT_MSG]);
						coms.send_msg = [0, 0,  READY_FOR_TRIAL_MSG]; # this just preps for the next heartbeat

					elif coms.recv_msg[TRIAL_IDX] == 5 and coms.recv_msg[MSG_IDX] == TRIAL_BEGIN_MSG:
						zero_boot(right_boot)
						# coms.send_msg[MSG_IDX] = 251
						coms.send_recv(send_msg = [0, 0,  TRIAL_EXIT_MSG]);
						coms.send_msg = [0, 0,  READY_FOR_TRIAL_MSG]; # this just preps for the next heartbeat
					
					
					elif coms.recv_msg[TRIAL_IDX] == 6 and coms.recv_msg[MSG_IDX] == TRIAL_BEGIN_MSG:
						left_boot.set_exo_current(0)
						right_boot.set_exo_current(0)
						
						left_boot.set_controller(CTRL_NONE)
						right_boot.set_controller(CTRL_NONE)
						
						# coms.send_msg[MSG_IDX] = 251
						coms.send_recv(send_msg = [0, 0,  TRIAL_EXIT_MSG]);
						coms.send_msg = [0, 0,  READY_FOR_TRIAL_MSG]; # this just preps for the next heartbeat	

					elif coms.recv_msg[TRIAL_IDX] == 7 and coms.recv_msg[MSG_IDX] == TRIAL_BEGIN_MSG:
						left_boot.set_exo_current(0)
						left_boot.set_controller(CTRL_NONE)
						
						# coms.send_msg[MSG_IDX] = 251
						coms.send_recv(send_msg = [0, 0,  TRIAL_EXIT_MSG]);
						coms.send_msg = [0, 0,  READY_FOR_TRIAL_MSG]; # this just preps for the next heartbeat

					elif coms.recv_msg[TRIAL_IDX] == 8 and coms.recv_msg[MSG_IDX] == TRIAL_BEGIN_MSG:
						right_boot.set_exo_current(0)
						right_boot.set_controller(CTRL_NONE)
						
						# coms.send_msg[MSG_IDX] = 251
						coms.send_recv(send_msg = [0, 0,  TRIAL_EXIT_MSG]);
						coms.send_msg = [0, 0,  READY_FOR_TRIAL_MSG]; # this just preps for the next heartbeat
					
					
					
					
					
					
					# Study 1 ------------------------------------------------------------------------------------
					
					
				elif coms.recv_msg[STUDY_IDX] == 1 :
					coms.send_msg[STUDY_IDX] = 1
					if coms.recv_msg[TRIAL_IDX] == 0 and coms.recv_msg[MSG_IDX] == TRIAL_BEGIN_MSG:
						try :
							if user_mass != -1 :
								s1 = STUDY1( user_mass, left_boot, right_boot)
								print('Study 1 initilized')
								# coms.send_msg[MSG_IDX] = 251
								coms.send_recv(send_msg = [1, 0,  TRIAL_EXIT_MSG]);
								coms.send_msg = [0, 0,  READY_FOR_TRIAL_MSG]; # this just preps for the next heartbeat
								time.sleep(3) # this is just to give me a chance to change values with packetsender
							else : 
								print('Study 1 initialization called but user mass not set')
						except IndexError :
							print('Study 1 initialization index error')
					elif coms.recv_msg[TRIAL_IDX] == 1 and coms.recv_msg[MSG_IDX] == TRIAL_BEGIN_MSG:
						# restart trial timer
						# start new recording file
						# left_boot.log_init()
						# right_boot.log_init()
						
						# startStopCapture(sync_led, left_boot, right_boot)
						# coms.send_recv(send_msg = [1,1, SYNC_FINISHED_BEGINNING_MSG])
						# time.sleep(.010) # pause for a bit to let messages pass
						# coms.send_recv(send_msg = [1,1, TRIAL_BEGIN_MSG])
						# coms.send_msg = [1, 1, CHECK_IN_MSG]
						print('Trial 1')
						trial_simple_start (sync_led, left_boot, right_boot,  coms, 1, 1)
						trial_running = s1.trial1(restart_trial = True)
						while ( trial_running and (coms.recv_msg[MSG_IDX] != STOP_CURRENT_TRIAL_MSG) and (coms.recv_msg[MSG_IDX] != PROGRAM_EXIT_MSG)) :
							#print('S1T1 : in Loop')
							time.sleep(1/(2*left_boot.frequency))
							sync_led.check()
							coms.heartbeat()
							trial_running = s1.trial1()
						
						trial_simple_end (sync_led, left_boot, right_boot,  coms, 1, 1)
						# coms.send_recv(send_msg = [1, 1,  TRIAL_EXIT_MSG]);
						# coms.send_msg = [1, 1, CHECK_IN_MSG]
						# # wait for the treadmill to stop if program exit received go strait to exit
						# while coms.recv_msg[TRIAL_IDX] != TREADMILL_STOP_MSG and (coms.recv_msg[TRIAL_IDX] != PROGRAM_EXIT_MSG):
							# coms.heartbeat()
						
						# startStopCapture(sync_led, left_boot, right_boot)
						# coms.send_recv(send_msg = [1, 1,  SYNC_FINISHED_END_MSG]);
						# sync_led.set_state(1)
						
						# coms.send_msg = [0, 0,  READY_FOR_TRIAL_MSG]; # this just preps for the next
						
					elif coms.recv_msg[TRIAL_IDX] == 2 and coms.recv_msg[MSG_IDX] == TRIAL_BEGIN_MSG:
						print('Trial 2')
						# restart trial timer
						# start new recording file
						# left_boot.data_file = left_boot.log_init()
						# right_boot.data_file = right_boot.log_init()
						
						# startStopCapture(sync_led, left_boot, right_boot)
						# coms.send_recv(send_msg = [1,2, SYNC_FINISHED_BEGINNING_MSG])
						# time.sleep(.010) # pause for a bit to let messages pass
						# coms.send_recv(send_msg = [1,2, TRIAL_BEGIN_MSG])
						# coms.send_msg = [1, 1, CHECK_IN_MSG]

						trial_simple_start (sync_led, left_boot, right_boot,  coms, 1, 2)
						trial_running = s1.trial2(restart_trial = True)
						
						
						while ( trial_running and (coms.recv_msg[MSG_IDX] != STOP_CURRENT_TRIAL_MSG) and (coms.recv_msg[MSG_IDX] != PROGRAM_EXIT_MSG)) :
							#print('S1T2 : in Loop')
							time.sleep(1/(2*left_boot.frequency))
							sync_led.check()
							coms.heartbeat()
							trial_running = s1.trial2()
						trial_simple_end (sync_led, left_boot, right_boot, coms, 1, 2)
						
						# coms.send_recv(send_msg = [1, 2,  TRIAL_EXIT_MSG]);
						# coms.send_msg = [1, 1, CHECK_IN_MSG]
						# # wait for the treadmill to stop if program exit received go strait to exit
						# while coms.recv_msg[TRIAL_IDX] != TREADMILL_STOP_MSG and (coms.recv_msg[TRIAL_IDX] != PROGRAM_EXIT_MSG):
							# coms.heartbeat()
						
						# startStopCapture(sync_led, left_boot, right_boot)
						# coms.send_recv(send_msg = [1, 2,  SYNC_FINISHED_END_MSG]);
						# sync_led.set_state(1)
						
						# coms.send_msg = [0, 0,  READY_FOR_TRIAL_MSG]; # this just preps for the next
						
					elif coms.recv_msg[TRIAL_IDX] == 3 and coms.recv_msg[MSG_IDX] == TRIAL_BEGIN_MSG:
						print('Trial 3')
						# restart trial timer
						# start new recording file
						# left_boot.data_file = left_boot.log_init()
						# right_boot.data_file = right_boot.log_init()
						
						# startStopCapture(sync_led, left_boot, right_boot)
						# coms.send_recv(send_msg = [1,3, SYNC_FINISHED_BEGINNING_MSG])
						# time.sleep(.010) # pause for a bit to let messages pass
						# coms.send_recv(send_msg = [1,3, TRIAL_BEGIN_MSG])
						# coms.send_msg = [1, 1, CHECK_IN_MSG]
						
						trial_simple_start (sync_led, left_boot, right_boot,  coms, 1, 3)
						trial_running = s1.trial3(restart_trial = True)
						
						while ( trial_running and (coms.recv_msg[MSG_IDX] != STOP_CURRENT_TRIAL_MSG) and (coms.recv_msg[MSG_IDX] != PROGRAM_EXIT_MSG)) :
							#print('S1T3 : in Loop')
							time.sleep(1/(2*left_boot.frequency))
							sync_led.check()
							coms.heartbeat()
							trial_running = s1.trial3()
						trial_simple_end (sync_led, left_boot, right_boot, coms, 1, 3)
						# coms.send_recv(send_msg = [1, 3,  TRIAL_EXIT_MSG]);
						# coms.send_msg = [1, 1, CHECK_IN_MSG]
						# # wait for the treadmill to stop if program exit received go strait to exit
						# while coms.recv_msg[TRIAL_IDX] != TREADMILL_STOP_MSG and (coms.recv_msg[TRIAL_IDX] != PROGRAM_EXIT_MSG):
							# coms.heartbeat()
						
						# startStopCapture(sync_led, left_boot, right_boot)
						# coms.send_recv(send_msg = [1, 3,  SYNC_FINISHED_END_MSG]);
						# sync_led.set_state(1)
						
						# coms.send_msg = [0, 0,  READY_FOR_TRIAL_MSG]; # this just preps for the next
					
					elif coms.recv_msg[TRIAL_IDX] == 4 and coms.recv_msg[MSG_IDX] == TRIAL_BEGIN_MSG:
						print('Trial 4')
						# restart trial timer
						# start new recording file
						# left_boot.data_file = left_boot.log_init()
						# right_boot.data_file = right_boot.log_init()
						
						# startStopCapture(sync_led, left_boot, right_boot)
						# coms.send_recv(send_msg = [1,2, SYNC_FINISHED_BEGINNING_MSG])
						# time.sleep(.010) # pause for a bit to let messages pass
						# coms.send_recv(send_msg = [1,2, TRIAL_BEGIN_MSG])
						# coms.send_msg = [1, 1, CHECK_IN_MSG]

						trial_simple_start (sync_led, left_boot, right_boot,  coms, 1, 4)
						trial_running = s1.trial4(restart_trial = True)
						
						
						while ( trial_running and (coms.recv_msg[MSG_IDX] != STOP_CURRENT_TRIAL_MSG) and (coms.recv_msg[MSG_IDX] != PROGRAM_EXIT_MSG)) :
							#print('S1T2 : in Loop')
							time.sleep(1/(2*left_boot.frequency))
							sync_led.check()
							coms.heartbeat()
							trial_running = s1.trial4()
						trial_simple_end (sync_led, left_boot, right_boot, coms, 1, 4)
						
						
					elif coms.recv_msg[TRIAL_IDX] == 5 and coms.recv_msg[MSG_IDX] == TRIAL_BEGIN_MSG:
						print('Trial 5')
						# restart trial timer
						# start new recording file
						# left_boot.data_file = left_boot.log_init()
						# right_boot.data_file = right_boot.log_init()
						
						# startStopCapture(sync_led, left_boot, right_boot)
						# coms.send_recv(send_msg = [1,2, SYNC_FINISHED_BEGINNING_MSG])
						# time.sleep(.010) # pause for a bit to let messages pass
						# coms.send_recv(send_msg = [1,2, TRIAL_BEGIN_MSG])
						# coms.send_msg = [1, 1, CHECK_IN_MSG]

						trial_simple_start (sync_led, left_boot, right_boot,  coms, 1, 5)
						trial_running = s1.trial5(restart_trial = True)
						
						
						while ( trial_running and (coms.recv_msg[MSG_IDX] != STOP_CURRENT_TRIAL_MSG) and (coms.recv_msg[MSG_IDX] != PROGRAM_EXIT_MSG)) :
							#print('S1T2 : in Loop')
							time.sleep(1/(2*left_boot.frequency))
							sync_led.check()
							coms.heartbeat()
							trial_running = s1.trial5()
						trial_simple_end (sync_led, left_boot, right_boot, coms, 1, 5)
				# Study 2 ------------------------------------------------------------------------------------
				
				
				left_boot.set_exo_current(0)
				right_boot.set_exo_current(0)
				
				time.sleep(.03)
				
				left_boot.set_controller(CTRL_NONE)
				right_boot.set_controller(CTRL_NONE)
	
				
				time.sleep(1/(2*left_boot.frequency))
				#print(str(left_boot.data_current[left_boot.idx_time]) + "\t" + str(time.monotonic() * 1000))
				
				###################################################
								
#				if len(left_boot.data_que) == 50 :
					
#					pred_torque_sequence = torque_pred_pipeline.run(left_boot.data_que, right_boot.data_que)
					
#					print("torque estimate : " + str(pred_torque_sequence[0]))
					
					#clearTerminal()
					#printData(left_boot.labels_current, left_boot.data_current)
					#printData(right_boot.labels_current, right_boot.data_current)
					
					
					
					# commanding 1 Nm = 1000 Nmm , set exo current takes mA and ankle_torque_to_current outputs A.
#					left_boot.set_exo_current(left_boot.ankle_torque_to_current(pred_torque_sequence[0]*50*1000)*1000) # pred torque is currently 0-1 shaping it to 50000 Nmm
					#right_boot.set_exo_current(right_boot.ankle_torque_to_current(1000)*1000)
				
				###################################################
				
				
				
				
				
				# commanding 1 Nm = 1000 Nmm , set exo current takes mA and ankle_torque_to_current outputs A.
#					
				# left_boot.set_exo_current(left_boot.ankle_torque_to_current(0)*1000)
				# right_boot.set_exo_current(right_boot.ankle_torque_to_current(0)*1000)
				
				
				
				
				#left_boot.set_exo_current(2500*sync_led.state)
				#right_boot.set_exo_current(2500*sync_led.state)
				
				#left_boot.set_exo_current(2500)
				#right_boot.set_exo_current(2500)
				
				
				
				
				
				
		except KeyboardInterrupt:
			print("KeyboardInterrupt has been caught.")


		# except IndexError:
			# print('Index out of range')



	
	
	
	
	
	
	
	# sync_led.set_state(0)
	
	left_boot.set_exo_current(0)
	right_boot.set_exo_current(0)
	
	time.sleep(.03)
	
	left_boot.set_controller(CTRL_NONE)
	right_boot.set_controller(CTRL_NONE)
	
	# startStopCapture(sync_led, left_boot, right_boot)
	
	# sync_led.set_state(0)
	# left_boot.read_data()
	# right_boot.read_data()
	# time.sleep(1)
	# left_boot.read_data()
	# right_boot.read_data()
	# time.sleep(.001)
	
	# sync_led.set_state(1)
	# left_boot.read_data()
	# right_boot.read_data()
	# time.sleep(2)
	# left_boot.read_data()
	# right_boot.read_data()
	# time.sleep(.001)
	
	# sync_led.set_state(0)
	# left_boot.read_data()
	# right_boot.read_data()
	# time.sleep(1)
	# left_boot.read_data()
	# right_boot.read_data()
	
	
	del left_boot
	del right_boot
	
	cleanupPlanStack()
	
	

main()
