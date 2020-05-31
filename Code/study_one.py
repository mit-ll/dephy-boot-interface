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


# Cuing Study One Code
# this is to test the setup for the trials using commands over TCP
import time
import numpy as np
# Create class for study
 
 
 
 
class STUDY1 : 
	# initialization with users mass
	def __init__ (self, user_mass, left_boot, right_boot) :
		self.user_mass = user_mass 	# store the users mass
		self.left_boot = left_boot		# store the reference to the left boot
		self.right_boot = right_boot	# store the reference to the right boot
		self.start_time = time.monotonic()	# initialize the time for keeping track of when each trial started, this will be overwritten when the trial starts.
		rspg = 0
		opg = 27.1
		ppg = 52.4
		spg = 62.7
		ot = 2
		npt = .175
		
		self.left_boot.init_collins_profile(mass = self.user_mass, ramp_start_percent_gait = rspg, onset_percent_gait = opg, peak_percent_gait = ppg, stop_percent_gait = spg, onset_torque = ot, normalized_peak_torque = npt)	# initialize the Zhang/Collins profile
		self.right_boot.init_collins_profile(mass = self.user_mass, ramp_start_percent_gait = rspg, onset_percent_gait = opg, peak_percent_gait = ppg, stop_percent_gait = spg, onset_torque = ot, normalized_peak_torque = npt)	# initialize the Zhang/Collins profile
		
		# self.left_boot.init_collins_profile(mass = self.user_mass, ramp_start_percent_gait = 0, onset_percent_gait = 27.1, peak_percent_gait = 52.4, stop_percent_gait = 62.7, onset_torque = 2, normalized_peak_torque = .175)	# initialize the Zhang/Collins profile
		# self.right_boot.init_collins_profile(mass = self.user_mass, ramp_start_percent_gait = 0, onset_percent_gait = 27.1, peak_percent_gait = 52.4, stop_percent_gait = 62.7, onset_torque = 2, normalized_peak_torque = .175)	# initialize the Zhang/Collins profile
		
	
	# Handles the running of the specific trials segment
	def trial_handler(self, tactor_percent, use_torque) :
				# update tactor value
		self.left_boot.tactor_trigger_percent = tactor_percent	# set the percent of gait the tactor should trigger
		self.right_boot.tactor_trigger_percent = tactor_percent	# set the percent of gait the tactor should trigger
					
		# read_data
		self.left_boot.read_data()	# read in the new data and trigger the tactor if appropriate
		self.right_boot.read_data()	# read in the new data and trigger the tactor if appropriate
		# if use_torque
		#	run collins
		if use_torque == 1 :
			self.left_boot.run_collins_profile(external_read = True)	# apply the torque appropriate to the Zhang/Collins profile
			self.right_boot.run_collins_profile(external_read = True)	# apply the torque appropriate to the Zhang/Collins profile
		# else 
		#	send 0
		else :
			# can make this go_slack once that is worked out
			self.left_boot.zero_current()	# set the control mode to current and set the current command to 0, i.e. no torque
			self.right_boot.zero_current()	# set the control mode to current and set the current command to 0, i.e. no torque
			
			
	# create timing check
	def check_time(self, segments_minutes,  restart_trial = False) :
		segments_seconds= [60 * x for x in segments_minutes]	# convert the segment time from minutes to seconds
		time_points		=	np.cumsum(segments_seconds)	# get the segment times from the trial start not the time for the individual segment
		if restart_trial :
			self.start_time = time.monotonic()	# reset the start time
		current_time = time.monotonic()			# get the current time
		time_elapsed = current_time - self.start_time	# get the elapsed time
		#print('time elapsed : ' + str(time_elapsed))
		time_passed = (time_elapsed) > time_points  # get a logical list of the time points that have passed
		#print(time_passed)
		not_passed_idx = [i for i, val in enumerate(time_passed) if not val]	# get the index values of the time points that have not passed.
		#print(not_passed_idx)
		if not_passed_idx :	# check if there are any values in the list
			current_idx = not_passed_idx[0]	# if there are the first value is the largest segment whose timepoint hasn't passed
		else :
			current_idx = -1	# if all the points have passed the trial has ended
		return current_idx	# return the index of the current trial
		
		
	 # Trial 1
	def trial1 (self, restart_trial = False) :
		#TODO: Don't cut while torque is high
		# segment_time	=	[.1,.1,.1]		# Short Trial for testing 
		segment_time	=	[2,	12,		2]		# time in minutes for each part of the trial 
		tactor_percent	=	[-1,	-1,	-1]	# percent of gait the tactor should trigger at for each part of the trial, -1 for no tactor 
		use_torque 		=	[ 0,	1,		0]		# whether or not to use tactor for each part of the trial, 1 to use torque, 0 for no torque
		trial_running = True	# return value to let the external script know if the trial is over.
		current_idx = self.check_time(segment_time, restart_trial = restart_trial)	# Check which part of the trial we are currently in
		
		if restart_trial :
			self.left_boot.clear_gait_estimate()
			self.right_boot.clear_gait_estimate()
		

		# Test this and comment out the below if statement
		if current_idx != -1 :	# Check if the trial is still running
			self.trial_handler(tactor_percent[current_idx], use_torque[current_idx])	# handle the commands for the trial based on the settings
		
		# if current_idx != -1 :
			# # update tactor value
			# self.left_boot.tactor_trigger_percent = tactor_percent[current_idx]
			# self.right_boot.tactor_trigger_percent = tactor_percent[current_idx]
						
			# # read_data
			# self.left_boot.read_data()
			# self.right_boot.read_data()
			# # if use_torque
			# #	run collins
			# if use_torque[current_idx] == 1 :
				# self.left_boot.run_collins_profile(external_read = True)
				# self.right_boot.run_collins_profile(external_read = True)
			# # else 
			# #	send 0
			# else :
				# # can make this go_slack once that is worked out
				# self.left_boot.zero_current()
				# self.right_boot.zero_current()
			
			# #print('trial running')
			
			# print the current state just to track
			# print('idx : ' + str(current_idx))
			# print('tactor percent : ' + str(tactor_percent[current_idx]))
			# print('Use torque : ' + str(use_torque[current_idx]))
		else :
			# if the trial is over set the current to zero and tell the return that the trial has finished
			self.left_boot.zero_current()
			self.right_boot.zero_current()
			trial_running = False
			print('trial finished')
			
		return trial_running
	
	
	# Trial 2
	def trial2 (self, restart_trial = False) :
		# !!!for comments see trial 1
		segment_time	=	[2,	1,		1,		1,		1,		1,		1,		1,		1,		1,		1,		1,		1,		1,		1,		1,		1,		2]
		tactor_percent	=	[-1,	90,	95,	0,		5,		10,	15,	20,	25,	25,	20,	15,	10,	5,		0,		95,	90,	-1]
		use_torque 		=	[ 0,	0,		0,		0,		0,		0,		0,		0,		0,		0,		0,		0,		0,		0,		0,		0,		0,		0]
		trial_running = True
		current_idx = self.check_time(segment_time, restart_trial = restart_trial)
		
		if restart_trial :
			self.left_boot.clear_gait_estimate()
			self.right_boot.clear_gait_estimate()
		
		# Test this and comment out the below if statement
		if current_idx != -1 :
			self.trial_handler(tactor_percent[current_idx], use_torque[current_idx])
			
		# if current_idx != -1 :
			# # update tactor value
			# self.left_boot.tactor_trigger_percent = tactor_percent[current_idx]
			# self.right_boot.tactor_trigger_percent = tactor_percent[current_idx]
						
			# # read_data
			# self.left_boot.read_data()
			# self.right_boot.read_data()
			# # if use_torque
			# #	run collins
			# if use_torque[current_idx] == 1 :
				# self.left_boot.run_collins_profile(external_read = True)
				# self.right_boot.run_collins_profile(external_read = True)
			# # else 
			# #	send 0
			# else :
				# # can make this go_slack once that is worked out
				# self.left_boot.zero_current()
				# self.right_boot.zero_current()
			
			# #print('trial running')
			
			
			# print('idx : ' + str(current_idx))
			# print('tactor percent : ' + str(tactor_percent[current_idx]))
			# print('Use torque : ' + str(use_torque[current_idx]))
		else :
			trial_running = False
			print('trial finished')
			
		return trial_running


	# Trial 3
	def trial3 (self, restart_trial = False) :
		# !!!for comments see trial 1
		segment_time	=	[2,	1,		1,		1,		1,		1,		1,		1,		1,		1,		1,		1,		1,		1,		1,		1,		1,		2]
		tactor_percent	=	[-1,	25,	20,	15,	10,	5,		0,		95,	90,	90,	95,	0,		5,		10,	15,	20,	25,	-1]
		use_torque 		=	[ 0,	0,		0,		0,		0,		0,		0,		0,		0,		0,		0,		0,		0,		0,		0,		0,		0,		0]
		
		trial_running = True
		current_idx = self.check_time(segment_time, restart_trial = restart_trial)
		
		if restart_trial :
			self.left_boot.clear_gait_estimate()
			self.right_boot.clear_gait_estimate()
		
		# Test this and comment out the below if statement
		if current_idx != -1 :
			self.trial_handler(tactor_percent[current_idx], use_torque[current_idx])
		
		else :
			trial_running = False
			print('trial finished')
			
		return trial_running 
		
		
	# Trial 4
	def trial4 (self, restart_trial = False) :
		# !!!for comments see trial 1
		segment_time	=	[2,	1,		1,		1,		1,		1,		1,		1,		1,		1,		1,		1,		1,		1,		1,		1,		1,		1,		2]
		tactor_percent	=	[-1,	-1,	90,	95,	0,		5,		10,	15,	20,	25,	25,	20,	15,	10,	5,		0,		95,	90,	-1]
		use_torque 		=	[ 0,	1,		1,		1,		1,		1,		1,		1,		1,		1,		1,		1,		1,		1,		1,		1,		1,		1,		0]
		trial_running = True
		current_idx = self.check_time(segment_time, restart_trial = restart_trial)
		
		if restart_trial :
			self.left_boot.clear_gait_estimate()
			self.right_boot.clear_gait_estimate()
		
		# Test this and comment out the below if statement
		if current_idx != -1 :
			self.trial_handler(tactor_percent[current_idx], use_torque[current_idx])
		
		# if current_idx != -1 :
			# # update tactor value
			# self.left_boot.tactor_trigger_percent = tactor_percent[current_idx]
			# self.right_boot.tactor_trigger_percent = tactor_percent[current_idx]
						
			# # read_data
			# self.left_boot.read_data()
			# self.right_boot.read_data()
			# # if use_torque
			# #	run collins
			# if use_torque[current_idx] == 1 :
				# self.left_boot.run_collins_profile(external_read = True)
				# self.right_boot.run_collins_profile(external_read = True)
			# # else 
			# #	send 0
			# else :
				# # can make this go_slack once that is worked out
				# self.left_boot.zero_current()
				# self.right_boot.zero_current()
				
			
			# #print('trial running')
			
			
			# print('idx : ' + str(current_idx))
			# print('tactor percent : ' + str(tactor_percent[current_idx]))
			# print('Use torque : ' + str(use_torque[current_idx]))
			
		else :
			trial_running = False
			print('trial finished')
			
		return trial_running
	
	
	
 
	# Trial 5
	def trial5 (self, restart_trial = False) :
		# !!!for comments see trial 1
		segment_time	=	[2,	1,		1,		1,		1,		1,		1,		1,		1,		1,		1,		1,		1,		1,		1,		1,		1,		1,		2]
		tactor_percent	=	[-1,	-1,	25,	20,	15,	10,	5,		0,		95,	90,	90,	95,	0,		5,		10,	15,	20,	25,	-1]
		use_torque 		=	[ 0,	1,		1,		1,		1,		1,		1,		1,		1,		1,		1,		1,		1,		1,		1,		1,		1,		1,		0]
		trial_running = True
		current_idx = self.check_time(segment_time, restart_trial = restart_trial)
		
		if restart_trial :
			self.left_boot.clear_gait_estimate()
			self.right_boot.clear_gait_estimate()
		
		# Test this and comment out the below if statement
		if current_idx != -1 :
			self.trial_handler(tactor_percent[current_idx], use_torque[current_idx])
			
		else :
			trial_running = False
			print('trial finished')
			
		return trial_running
 
if __name__ == "__main__":
	# Used when we are running this independently to test parts
	s1 = STUDY1()	# create an instance of the study

	trial_running = s1.trial1(restart_trial = True)	# reset trial one
	while trial_running :	# while it is running 
		time.sleep(1)	# put in a pause to help view that things are working
		trial_running = s1.trial1()	# operate the trial.

