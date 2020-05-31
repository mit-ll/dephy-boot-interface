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
from builtins import input

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

pardir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(pardir)
from fxUtil import *
from tlc2543_def import *
from tactor_defs import *

#from readData import *
from collections import deque
import math as m
from datetime import datetime
import configparser

# trying to follow the https:#www.python.org/dev/peps/pep-0008/ style

CURRENT_LIMIT = 25000 # mA : Current limit for the motor command.  You should still set the values in boots protection circuit because the PI controller and position controller can still push it higher.

QUE_LENGTH = 50 # number of samples to use for the LSTM inferance.



LEFT = 1	# modifies values for the left boot
RIGHT = -1	# modifies values for the right boot

# this was used with c since python handles types itself we will just make this one.
FIXED_POINT_COEFF_DEG = 1
def FIXED_POINT_CONVERSION_DEG(angle)	:
	return (angle)/FIXED_POINT_COEFF_DEG

# this was used with c since python handles types itself we will just make this one.
FIXED_POINT_COEFF_MM = 1
def FIXED_POINT_CONVERSION_MM(distance) :
	return (distance) / FIXED_POINT_COEFF_MM

# this was used with c since python handles types itself we will just make this one.
FIXED_POINT_COEFF_ACCL = 1
def FIXED_POINT_CONVERSION_ACCL(acceleration) :
	return (acceleration) / FIXED_POINT_COEFF_ACCL

# this was used with c since python handles types itself we will just make this one.
FIXED_POINT_COEFF_GYRO  = 1
def FIXED_POINT_CONVERSION_GYRO(angularVelocity) :	
	return (angularVelocity) / FIXED_POINT_COEFF_GYRO

TICKS_TO_ANGLE_COEFF =  0.02197 * FIXED_POINT_COEFF_DEG			# converts encoder ticks to degrees
ANGLE_TO_TICKS_COEFF = 1/TICKS_TO_ANGLE_COEFF					# converts angles to moter ticks
BIT_TO_ACCL_COEFF	= 1 / 8192 * 9.8 * FIXED_POINT_COEFF_ACCL	# converts accelerometer readings to acceleration m/s^2
BIT_TO_GYRO_COEFF	= 1 / 32.8 * FIXED_POINT_COEFF_GYRO			# converts gyro reading to angular velocity deg/s


SLACK_STEP_SIZE = (3 * ANGLE_TO_TICKS_COEFF)	# this limits how fast the slack position can change.

MAX_FILE_SIZE = 500 * 10**6 # 500 MB : limits the file size writting so you don't have to open a really big file

NUM_GAIT_TIMES_TO_AVERAGE = 3	# for the gait duration estimate how many gait cycles to average.  More changes slower but is more stable.  This might become obsolete if we move to an infinate horizon filter.

LEFT_Tactor_PIN = 7		# pin to use to trigger left tactor
RIGHT_Tactor_PIN = 15	# pin to use to trigger right tactor

ARMED_DURATION_PERCENT = 10

NO_SLACK_CURRENT = 400

# convert degrees to radians
def DEG_TO_RAD(angle_deg)  :
	return (angle_deg) * m.pi / 180
	
# convert mNm or Nmm to Nm
def NMM_TO_NM(torque)  :
	return (torque) / 1000

# convert Nm to mNm or Nmm 
def NM_TO_NMM(torque)  :
	return (torque) * 1000

# convert A to mA	
def A_TO_MA(torque)  :
	return (torque) * 1000

# convert mA to A
def MA_TO_A(torque)  :
	return (torque) / 1000


#Number of degrees for the motor to move in one step converted to ticks, when moving to a slack position
#define SLACK_STEP_SIZE (1 * ANGLE_TO_TICKS_COEFF)

#
# Class to interface with the boot
#
class ExoBoot:
	
	#
	# Initialize the instance
	# inputs:
	# 	devSide		: see values above or use LEFT or RIGHT
	#	port		: port to use for communication
	#	baudRate	: Baud rate for the serial communication with the boot
	#	idx			: Idx of the port
	#	frequency	: Update frequency for the boot (how often it puts data in the buffer
	#	shouldLog	: Should the instance write data to a csv file
	#	shouldExoLog: Should the boots program log data independently
	#	shouldAuto	: Should the boot automatically add data to the buffer (true), if false it will only do that when requested.
	#	sync_pin	: Which pin is use for the sync led
	#	mode		: What pin refrence is used "BOARD" is more compatable between the RPi and other boards 
	#	use_tactor	: Should a tactor be used.  This is convenient to make false if you just want to test the boots without the peripheral board.
	#
	def __init__ (self, devSide, port, baudRate, idx, frequency = 1000,  shouldLog = True, shouldExoLog = False, shouldAuto = 1, sync_pin = 40, mode = "BOARD", use_tactor = True):
		# Attach the values to the instance.
		print('boot constructor : top')
		self.port = port
		self.baud_rate = baudRate
		self.idx = idx
		self.side = devSide
		
		self.frequency = frequency
		self.should_log = shouldLog
		self.should_exo_log = shouldExoLog
		self.should_auto = shouldAuto
		self.mode = CTRL_NONE
		self.use_tactor = use_tactor
		
		
		# Zhang/Collins parameters
		self.t0 = -1		# ramp start percent
		self.t1 = -1		# ramp end percent
		self.t2 = -1		# peak percent
		self.t3 = -1		# drop percent
		self.ts = -1		# top torque for the ramp
		self.tp = -1		# peak torque
		self.user_mass = -1	# user mass
		self.peak_torque_normalized = -1	# torque normalized to user mass
		
		# parameters for the Zhang/Collins torque curve
		self.a1 = -1
		self.b1 = -1
		self.c1 = -1
		self.d1 = -1
		
		self.a2 = -1
		self.b2 = -1
		self.c2 = -1
		self.d2 = -1
		
		
		self.segmentation_trigger = False	# goes high when heelstrike detected
		self.heelstrike_armed = False		# high when trigger armed
		self.segmentation_arm_threashold = 150 * FIXED_POINT_COEFF_GYRO	# the threashold that must be gone above to arm the trigger
		self.segmentation_trigger_threashold = 0	# the theashold that must be dropped below to trigger the heelstrike
		
		self.past_gait_times = [-1] * NUM_GAIT_TIMES_TO_AVERAGE	# store the most recent gait times
		
		self.expected_duration = -1	# current estimated gait duration
		
		self.heelstrike_timestamp_current = -1	# Timestamp of the most recent heelstrike
		self.heelstrike_timestamp_previous = -1	# Timestamp of the second most recent heelstrike
		self.armed_timestamp = -1		# timestamp of when the trigger was armed
		self.percent_gait = -1			# estimate of the percent of gait
		self.percent_gait_previous = -1	# the previous loops estimate of the percent gait. used to determine crossing of timepoints
		
		self.tactor_trigger_percent = -1	# percent of gait when the tactor should trigger
		self.tactor_trigger = False			# did the tactor trigger this loop
		if self.use_tactor :				# Initialize the tactor if we are using it
			self.tactor = Tactor(pin = (LEFT_Tactor_PIN if self.side == LEFT else RIGHT_Tactor_PIN))
			
		self.current_cmd = None	# how much current are we requesting
		
		# if you want things to be easy between the Nano and Pi you should use board otherwise it looks like the numbering system is different.
		self.mode = GPIO.getmode()	# read the current pin mode
		if self.mode == None :  # if it is not already set set it
			self.mode = GPIO.BOARD if mode == "BOARD" else GPIO.BCM
			GPIO.setmode(self.mode)
		self.sync_pin = sync_pin # this is just for reading
		
		# get the labels of the values we are streaming from the boot
		self.labels_stream = ["State time", 	\
			"Accel X", 	"Accel Y", 	"Accel Z", 	\
			"Gyro X", 	"Gyro Y",	"Gyro Z", 	\
			"Motor angle", "motor velocity",   "motor acceleration",	\
			"Motor voltage", "Motor current",	\
			"Battery voltage", "Battery current", \
			"Ankle angle", "Ankle vel"
		]

		# store the appropriate numbers for reading those values from the boot, check fxUtil.py for details
		self.vars_to_stream = [ 		\
			FX_STATETIME, 		\
			FX_ACCELX,	FX_ACCELY,	FX_ACCELZ, 	\
			FX_GYROX,  	FX_GYROY,  	FX_GYROZ,	\
			FX_ENC_ANG,	FX_ENC_VEL,	FX_ENC_ACC,	\
			FX_MOT_VOLT, FX_MOT_CURR,	\
			FX_BATT_VOLT, FX_BATT_CURR, \
			FX_ANKLE_ANG, FX_ANKLE_ANG_VEL
		]

		# store the index values for accessing the specific variables
		self.idx_time = 0
		self.idx_accl_x = 1
		self.idx_accl_y = 2
		self.idx_accl_z = 3
		self.idx_gyro_x = 4
		self.idx_gyro_y = 5
		self.idx_gyro_z = 6
		self.idx_motor_angle = 7
		self.idx_motor_vel = 8
		self.idx_motor_accl = 9
		self.idx_motor_voltage = 10
		self.idx_motor_current = 11
		self.idx_batt_voltage = 12
		self.idx_batt_current = 13
		self.idx_ankle_angle = 14
		self.idx_ankle_vel = 15
		self.idx_other_base = 16
		# EMG values from the ADC
		self.idx_emg_base = 0
		self.idx_emg_0 = self.idx_other_base + self.idx_emg_base
		self.idx_emg_1 = self.idx_other_base + self.idx_emg_base + 1
		self.idx_emg_2 = self.idx_other_base + self.idx_emg_base + 2
		self.idx_emg_3 = self.idx_other_base + self.idx_emg_base + 3
		# Possible additional IMU
		self.idx_aux_imu_base = 4
		self.idx_aux_accl_x = self.idx_other_base + self.idx_aux_imu_base
		self.idx_aux_accl_y = self.idx_other_base + self.idx_aux_imu_base + 1
		self.idx_aux_accl_z = self.idx_other_base + self.idx_aux_imu_base + 2
		self.idx_aux_gyro_x = self.idx_other_base + self.idx_aux_imu_base + 3
		self.idx_aux_gyro_y = self.idx_other_base + self.idx_aux_imu_base + 4
		self.idx_aux_gyro_z = self.idx_other_base + self.idx_aux_imu_base + 5
		# Control system info
		self.idx_sync = self.idx_aux_gyro_z + 1
		self.idx_percent_gait = self.idx_sync + 1
		self.idx_heelstrike_armed = self.idx_sync + 2
		self.idx_segmentation_trigger = self.idx_sync + 3
		self.idx_expected_duration = self.idx_sync + 4
		self.idx_tactor_trigger = self.idx_sync + 5
		self.idx_current_cmd = self.idx_sync + 6
		self.idx_torque_cmd = self.idx_sync + 7
		
		# store the channels that the boot should read from the ADC
		if (self.side == LEFT) :
			self.emg_channels = [0, 1, 2, 3]
		else :
			#self.emg_channels = [0, 1, 2, 3]
			self.emg_channels = [4, 5, 6, 7]
		# Initialize the ADC	
		self.adc = TLC2543()
		
		print('boot constructor : connecting to device')
		self.devId = self._connectToDevice()	# connect to the boot
		print('boot constructor : device connected : id : ' , self.devId)		
		fxSetStreamVariables(self.devId,self.vars_to_stream)	# set the variables to stream
		if not fxStartStreaming(self.devId,self.frequency,self.should_exo_log,self.should_auto): # start streaming
			raise Exception('Streaming failed')
		else:
			sleep(0.4)
		
		# Values used for zeroing the encoder position				
		self.motorTicksOffset = 0
		self.ankle_ticks_offset = 0
		
		# read in calibration data
		cal_filename = 'bootCal.txt'	# this file should exist in the directory
		config = configparser.ConfigParser()	# create the parser
		config.read(cal_filename)				# read in the file
		self.boot_id = config.get('ids', 'left' if self.side == LEFT else 'right')	# get the boot id number for the left or the right boot.  You should update these numbers in the config file for the boots you are using.
		self.ankle_ticks_abs_offset_plantar = config.getint(self.boot_id, 'ankle_reading_55_deg')	# get encoder reading at full plantar flexion, this was used for absolute position, not subject based position. Not currently used with the current method of calcuating the torque
		self.ankle_ticks_abs_offset = self.ankle_ticks_abs_offset_plantar - self.side * 55 * ANGLE_TO_TICKS_COEFF	# shift the plantar flexed position to a neutral ankle position
		# read in the values for the fit curve of the derivitave of the motor angle with respect to the ankle angle
		self.wm_wa_coeffs = [config.getfloat(self.boot_id,'poly4'), config.getfloat(self.boot_id,'poly3'), config.getfloat(self.boot_id,'poly2'), config.getfloat(self.boot_id,'poly1'),  config.getfloat(self.boot_id,'poly0')]
		# variable to store the value of the current derivitave
		self.wm_wa = 0 
		
		# ankle_cal_file = open(ankle_cal_filename, "r")
		# ankle_cal_text = ankle_cal_file.readlines()
		# self.ankle_ticks_abs_offset_plantar = int(ankle_cal_text[0 if self.side == LEFT else 1].rstrip("\n"))
		# self.ankle_ticks_abs_offset = self.ankle_ticks_abs_offset_plantar - self.side * 55 * ANGLE_TO_TICKS_COEFF 
		
		
		
		#=======================================================
		# physical properties
		
		# r[0] = min radius of the motor output shaft
		# r[1] = radius of the motor output shaft where the strap exits, this accounts for strap wrapping
		# r[2] = radius of the idler pulley
		# r[3] = radius of the end of the lever connected to the ankle
		
		self.r = [3.5 , 0, 6.4, 5.9]; # { 35 , 0, 63.5, 59};  # mm / fixed point coeff  initially this is 1 since this is python, r[1] is dependent on the motor angle and is set when that is read
		
		# d[0] = vertical distance from ankle joint to motor center
		# d[1] = distance from idler pulley to motor center
		# d[2] = horizontal distance from ankle joint to motor center
		# d[3] = length from ankle joint to center arc at the end of the ankle lever
		# d[4] = strap thickness
		self.d = [164.6 , 66.0, 3.5, 95.0, .9]; # mm / fixed point coeff initially this is 1,  d5 is the strap thickness

		# phi[0] = acute angle between horizontal plantar plane and line formed by d[3]
		# phi[1] = acute angle between the vertical and line formed by d[1]
		self.phi = [75.0, 42.5]; # deg /fixed point coeff initially this is 1;

		self.kt = 48 #95;		# motor torque constant Nmm/A 95 is the torque constant given by Dephy.  Independent evaluation of our system found it to be a portion of that 48.
		self.motorJ = 0;	# 100 # motor rotor inertia kg*mm^2
		self.motorB = 0;	# motor coeff of dynamic friction Nmm/(deg/s)
		self.motorBs = 0;	# motor coeff of static friction Nmm
		
		
		
		#=======================================================
		# the labels of the other values we are writing that don't come from the boot
		self.labels_other = ["EMG 0", "EMG 1", "EMG 2", "EMG 3", \
			"AUX Accel X", 	"AUX Accel Y", 	"AUX Accel Z", 	\
			"AUX Gyro X", 	"AUX Gyro Y",	"AUX Gyro Z", \
			"Sync", "Percent Gait", "Heelstrike Armed", "Segmentation Trigger", "Expected Duration", "Tactor Trigger", "Current Command", "Torque Command"]
		
		# create a deque that the LSTM will use for its inference.
		self.data_que = deque(maxlen = QUE_LENGTH)
		self.data_exo = [-1] * len(self.vars_to_stream)	# fill in the data with -1 till we read in the values
		self.data_other = [-1] * len(self.labels_other) # fill in the data with -1 till we read in the values
		self.labels_current = self.labels_stream + self.labels_other	# combine the labels 
		
		self.data_current = self.data_exo + self.data_other	# combine the data
		
		self.data_file = None	# store the data file handle
		if self.should_log :	# if we are logging create the file
			self.file_base = "";
			self.file_extension = "";
			self.data_filename = "";
			
			# initilize the logging
			self.data_file = self.log_init()
		
		self.read_data()	# read in the current data from the system
		
		
		self.data_current = [0] * len(self.vars_to_stream) # clear this variable since the first read will not have all the zeroing set
		self.data_que.clear() # clear this variable since the first read will not have all the zeroing set
		
		
		
		#=======================================================
		
		# controller properties
		self.mode = 0
		self.currentKp = 0
		self.currentKi = 0
		
		self.positionKp = 0
		self.positionKi = 0
		
		self.impedanceKp = 0
		self.impedanceKi = 0
		self.impedanceStiffness = 0
		self.impedanceDamping = 0
		
		
		#=======================================================
	#
	# this is pulled from the Dephy examples
	#
	def _connectToDevice(self):
		fxOpen(self.port, self.idx, self.baud_rate)
		timeElapsed = 0
		TIMEOUT_LIMIT = 10
		while(timeElapsed <= TIMEOUT_LIMIT and not fxIsOpen(self.idx)):
			# There is certainly a better way to do this
			sleep(0.2)
			timeElapsed += 0.2

		if(not fxIsOpen(self.idx)):
			raise Exception("Couldn't connect to port {}".format(self.port))
		
		sleep(0.1)
		MAX_DEVICE_ID_ATTEMPTS = 10
		num_attempts = 0
		devIds = fxGetDeviceIds()
		while(num_attempts < MAX_DEVICE_ID_ATTEMPTS and (len(devIds) == 0 or len(devIds) < (self.idx+1))):  
		# added in a check because it was only recognizing the first idx when running it in a script.  When manually running it would error the first time but be ok the second time.
			sleep(0.2)
			devIds = fxGetDeviceIds()
			print ("attempt number : " + str(num_attempts))
		print("devIds : ")
		print (devIds)
		if len(devIds) == 0:
			raise Exception('Failed to get device Id')
		devId = devIds[self.idx]
		print("Devid is: ", devId)
		return devId
	
	
	#
	#	Update the current sensor information from the system
	#
	def read_data(self):
		self.data_exo =  fxReadDevice(self.devId,self.vars_to_stream)
			
		if (self.data_current[self.idx_time]  != self.data_exo[self.idx_time] ) : # update the data if new data has come in
			
			# !!! Make sure signs are correct.
			self.motorTicksRaw = self.data_exo[self.idx_motor_angle]
			self.motorTicksZeroed =  self.side * (self.motorTicksRaw - self.motorTicksOffset);  # remove the offset, and adjust for side
			self.motorTicksAbs = self.side * (self.motorTicksRaw - self.motorTicksOffset);  # remove the offset, and adjust for side
			try :  # TODO: add offset so it is current position + some buffer.
				self.motorSlackPosition = self.side * max(self.side * self.motorTicksRaw, self.side * self.motorSlackPosition);
			except AttributeError:
				self.motorSlackPosition = self.motorTicksRaw
			
			self.r[1] 		= self.r[0] + 2.5 * self.d[4] + FIXED_POINT_CONVERSION_DEG(DEG_TO_RAD(self.ticks_to_angle(self.motorTicksZeroed)))/(2 * m.pi)*self.d[4]  # Find the functional 
			
			
			self.ankleTicksRaw = self.data_exo[self.idx_ankle_angle]
			self.ankleTicksZeroed = self.side * (self.ankleTicksRaw - self.ankle_ticks_offset) # remove the offset, and adjust for side
			self.ankleTicksAbsZeroed = self.side * (self.ankleTicksRaw - self.ankle_ticks_abs_offset) # remove the offset, and adjust for side
			#self.gyro_raw = [self.data_exo(self.idx_gyro_x), self.data_exo(self.idx_gyro_y), self.data_exo(self.idx_gyro_z)]
			#self.accl_raw = [self.data_exo(self.idx_accl_x), self.data_exo(self.idx_accl_y), self.data_exo(self.idx_accl_z)]
			
			accl_gyro_rotated = self.rotate_imu(self.data_exo[self.idx_accl_x], self.data_exo[self.idx_accl_y], self.data_exo[self.idx_accl_z], self.data_exo[self.idx_gyro_x], self.data_exo[self.idx_gyro_y], self.data_exo[self.idx_gyro_z])
			
			self.data_current = [-1] * len(self.data_exo)  # clear it.  I will be extending the data_other to the end and I don't want issues.
			
			self.data_current[self.idx_time] = self.data_exo[self.idx_time]
			self.data_current[self.idx_accl_x] = accl_gyro_rotated[0]
			self.data_current[self.idx_accl_y] = accl_gyro_rotated[1]
			self.data_current[self.idx_accl_z] = accl_gyro_rotated[2]
			self.data_current[self.idx_gyro_x] = accl_gyro_rotated[3]
			self.data_current[self.idx_gyro_y] = accl_gyro_rotated[4]
			self.data_current[self.idx_gyro_z] = accl_gyro_rotated[5]
			self.data_current[self.idx_motor_angle] = self.ticks_to_angle(self.motorTicksZeroed)
			self.data_current[self.idx_motor_vel] = self.data_exo[self.idx_motor_vel]
			self.data_current[self.idx_motor_accl] = self.data_exo[self.idx_motor_accl] 
			self.data_current[self.idx_motor_voltage] = self.data_exo[self.idx_motor_voltage]
			self.data_current[self.idx_motor_current] = self.data_exo[self.idx_motor_current]
			self.data_current[self.idx_batt_voltage] = self.data_exo[self.idx_batt_voltage]
			self.data_current[self.idx_batt_current] = self.data_exo[self.idx_batt_current]
			self.data_current[self.idx_ankle_angle] = self.ticks_to_angle(self.ankleTicksZeroed)
			self.data_current[self.idx_ankle_vel] = self.data_exo[self.idx_ankle_vel]
	 
			
			self.data_other =[-1] * len(self.labels_other)  # initial place holder will be populated with EMG and IMU
			emg = self.adc.read_channels(self.emg_channels)
			#print("EMG length : " + str(len(emg)))
			self.data_other[self.idx_emg_0 - self.idx_other_base] = emg[self.idx_emg_0 - self.idx_emg_base - self.idx_other_base]
			self.data_other[self.idx_emg_1 - self.idx_other_base] = emg[self.idx_emg_1 - self.idx_emg_base - self.idx_other_base]
			self.data_other[self.idx_emg_2 - self.idx_other_base] = emg[self.idx_emg_2 - self.idx_emg_base - self.idx_other_base]
			self.data_other[self.idx_emg_3 - self.idx_other_base] = emg[self.idx_emg_3 - self.idx_emg_base - self.idx_other_base]
			# TODO: ADD IMU
			
			# Sync
			self.data_other[self.idx_sync - self.idx_other_base] = GPIO.input(self.sync_pin)
			
						
			
			self.check_for_heelstrike()
			
			if (self.segmentation_trigger) :
				self.heelstrike_timestamp_previous = self.heelstrike_timestamp_current
				self.heelstrike_timestamp_current = self.data_current[self.idx_time]
				self.update_expected_duration()
			
			self.percent_gait_previous = self.percent_gait
			self.percent_gait_calc()
			
			# TODO: tactor trigger based on percent gait
			if (-1 != self.tactor_trigger_percent) :
				if (((self.percent_gait_previous < self.tactor_trigger_percent) and (self.tactor_trigger_percent <= self.percent_gait)) or (self.tactor_trigger_percent == 0 and self.segmentation_trigger)) : 
					self.tactor_trigger = True
					if self.use_tactor :
						self.tactor.go()
				else :
					self.tactor_trigger = False
			
			self.calc_wm_wa()
			self.torque_cmd = (self.current_cmd/1000 * self.kt * self.wm_wa if self.current_cmd != None else None)
			
			# Gait Estimation Data
			self.data_other[self.idx_percent_gait - self.idx_other_base] = self.percent_gait
			self.data_other[self.idx_heelstrike_armed - self.idx_other_base] = (1 if self.heelstrike_armed else 0)
			self.data_other[self.idx_segmentation_trigger - self.idx_other_base] = (1 if self.segmentation_trigger else 0)
			self.data_other[self.idx_expected_duration - self.idx_other_base] = self.expected_duration
			self.data_other[self.idx_tactor_trigger - self.idx_other_base] = (1 if self.tactor_trigger else 0)
			self.data_other[self.idx_current_cmd - self.idx_other_base] = self.current_cmd 
			self.data_other[self.idx_torque_cmd - self.idx_other_base] = self.torque_cmd 
			
			self.data_current.extend(self.data_other)
			
			if self.should_log :
				self.log()
			
			self.data_que.append(self.data_current)
		
		
		
		
	# def defineGains (void); 					# default values all to 0
	# def defineGains (int, int, int);			# takes controller type and sets Kp and Ki for that controller
	# def defineGains (int, int, int, int, int);	# as above but also takes in the stiffness and damping for the impedance controller
	# def defineKB (int, int); # for the impedance mode set the stiffness and damping without touching the gains
	# def displayControllerState (void); # displays the current state
	# def exoControllerInit(void);
	
	
	def log_init(self) :
		
		start_time = datetime.now()
		time_str = start_time.strftime("%Y_%m_%d_%Hh%Mm%Ss")
		side_str = "_left" if self.side == LEFT else "_right"
		
		# you need to initilize these before calling these
		self.file_base = time_str + side_str
		self.file_extension = ".csv"
		
		self.data_filename = self.get_free_filename(self.file_base, self.file_extension)
		# cut start
		# self.data_filename = self.file_base + self.file_extension
		# print("filename : " + self.data_filename )
		# i = 0
		# while os.path.exists(self.data_filename):
			# i +=1
			# self.data_filename =  self.file_base + "_"+ str(i) + self.file_extension
		#  cut end		
		data_file = open(self.data_filename, 'a')
		
		labels_csv = ""
		for label in self.labels_current :
			labels_csv += label + ", "
		data_file.write(labels_csv)
		data_file.write("\n ")
		
		return data_file
	
	
	def get_free_filename(self, base, extension) :
		data_filename = base + extension
		i = 0
		while os.path.exists(data_filename):
			i +=1
			data_filename =  base + "_"+ str(i) + extension
		return data_filename
			
		
	def log (self) :
		if self.should_log :
			# check file size
			if os.path.getsize(self.data_filename) > MAX_FILE_SIZE :
				self.data_file.close()
				self.data_filename = self.get_free_filename(self.file_base, self.file_extension)
				self.data_file = open(self.data_filename, 'a')
				
				labels_csv = "";
				for label in self.labels_current :
					labels_csv += label + ", "
				self.data_file.write(labels_csv)
				self.data_file.write("\n ")
				
			for i in range(0, len(self.data_current)):
				self.data_file.write(str(self.data_current[i]) if self.data_current[i] != None else 'nan')  # write out the data.  If the type is None change to 'nan' so it is easy for matlab to parse. 
				self.data_file.write(",")
			self.data_file.write("\n ")
		else :
			print("exo_defs :: log(self) : \n\tYOU WANTED TO LOG VALUES BUT LOGGING DATA WAS NOT SELECTED")

	
	
	
	
	
	
	def rotate_imu (self, accl_x_raw, accl_y_raw, accl_z_raw, gyro_x_raw, gyro_y_raw, gyro_z_raw):
		angleOffset = 37 * m.pi / 180
		rotAngleZ = 0
		rotAngleX = 0
		
		# first rotate about the Z axis to get the x axis forward on the boot
		# then rotate about the new x axis to get the y axis pointing up

		if (self.side == LEFT) :
			rotAngleZ = m.pi - angleOffset
			rotAngleX = 0
		
		else :
			rotAngleZ = angleOffset - m.pi
			rotAngleX = m.pi

		acclX = 	BIT_TO_ACCL_COEFF * (accl_x_raw* m.cos(rotAngleZ) + \
						accl_y_raw * -m.sin(rotAngleZ) )
		acclY = 	BIT_TO_ACCL_COEFF * (accl_x_raw * m.sin(rotAngleZ) * m.cos(rotAngleX) + \
						accl_y_raw * m.cos(rotAngleZ) * m.cos(rotAngleX) + \
						accl_z_raw * -m.sin(rotAngleX))
		acclZ = 	BIT_TO_ACCL_COEFF * (accl_x_raw * m.sin(rotAngleZ) * m.sin(rotAngleX) + \
						accl_y_raw * m.cos(rotAngleZ) * m.sin(rotAngleX) + \
						accl_z_raw * m.cos(rotAngleX))

		gyroX = 	BIT_TO_GYRO_COEFF * (gyro_x_raw* m.cos(rotAngleZ) + \
						gyro_y_raw * -m.sin(rotAngleZ))
		gyroY = 	BIT_TO_GYRO_COEFF * (gyro_x_raw * m.sin(rotAngleZ) * m.cos(rotAngleX) + \
						gyro_y_raw * m.cos(rotAngleZ) * m.cos(rotAngleX) + \
						gyro_z_raw * -m.sin(rotAngleX))
		gyroZ = 	BIT_TO_GYRO_COEFF * (gyro_x_raw * m.sin(rotAngleZ) * m.sin(rotAngleX) + \
						gyro_y_raw * m.cos(rotAngleZ) * m.sin(rotAngleX) + \
						gyro_z_raw * m.cos(rotAngleX))

		return [acclX, acclY, acclZ, gyroX, gyroY, gyroZ]
	
	
	
	# # /*
	# # *	Record the motor and ankle position when the function is called
	# # */
	def zero_encoders(self):
		self.read_data();
		self.motorTicksOffset = self.motorTicksRaw;
		self.ankle_ticks_offset = self.ankleTicksRaw;

	def calc_wm_wa (self) :
		self.wm_wa = 5 * self.ankleTicksRaw ** 4 * self.wm_wa_coeffs[0] + 4 * self.ankleTicksRaw ** 3 * self.wm_wa_coeffs[1] + 3 * self.ankleTicksRaw ** 2 * self.wm_wa_coeffs[2] + 2 * self.ankleTicksRaw * self.wm_wa_coeffs[3] + self.wm_wa_coeffs[4]
		self.wm_wa = 1 if self.wm_wa <= .5 else self.wm_wa  # safety check to keep it from getting too large
			# print ('wm_wa = ' + str(wm_wa)) 
		
	def ankle_torque_to_current(self, torque, mode = 0):
		# takes in torque in Nmm
		
		#TODO: add safety limit for ankle angles that would cause a negative moment arm.
		#	1. Simple - set moment arm to a small positive value as it will only ever be small in this range.
		#	2. complex (proper way) do the math for the other method, this should just be a matter of switching signs, but not really sure if this is meaningful as the moment arm is very close to zero.  
		#	3. most proper do individual boot calibrations for this part as the torque geometry is not dependent on the attachment to the person, since we are using magnetic encoders they should be consistent between power cycles.
		# Currently 3 is used. tentative YAY! Still needs to be tested
		# Dephy seems to just do a mapping of motor angle to ankle angle.  This may be a better method as the strap length may not be consistent.
		if mode == 0  :
			
			
			
			# get the current based on the torque cmd and the system state
			current = (torque / self.wm_wa) / self.kt;
			# current = (torque / wm_wa + self.motorJ * self.data_current[self.idx_motor_accl]+ self.motorB * self.data_current[self.idx_motor_vel]+ self.motorBs) / self.kt;
		
		elif mode ==1 :
			# idx 0 is x component 1 is y component
			# all points relative the to ankle position
			t32 = [0] * 2;	# tangent point where the the strap comes off the ankle lever
			x4  = [0] * 2;	# point where the the strap crosses the line between p2 and p3
			p2  = [0] * 2;	# the center of the idler pulley
			p3  = [0] * 2;	# the center or the arc the strap wraps around on the ankle lever)
			strap_dir = [0] * 2;	# the unit vector for the strap coming off the ankle lever

			# determine the effective radius of the motor output accounting for strap wrapping
			self.r[1] = self.r[0] + 2.5 * self.d[4] + FIXED_POINT_CONVERSION_DEG(DEG_TO_RAD(self.ticks_to_angle(self.motorTicksZeroed))) * self.d[4] / (2 * m.pi);  # mm/fixed_point_coeff

			# d and phi were not zero indexed in the derivation so the indexes here are one smaller than those in the derivation, definition in the header file is correct.
			# p2 is idler pulley center from the ankle joint center
			p2[0] = self.d[2] - self.d[1] * m.sin(FIXED_POINT_CONVERSION_DEG(DEG_TO_RAD(self.phi[1])));
			p2[1] = self.d[0] - self.d[1] * m.cos(FIXED_POINT_CONVERSION_DEG(DEG_TO_RAD(self.phi[1])));
			
			#print(("Left" if self.side == LEFT else "Right") + " ankle angle : " + str(self.ticks_to_angle(self.ankleTicksAbsZeroed)))
			#print(("Left" if self.side == LEFT else "Right") + " p2 : " + str(p2))

			# p3 is radius center of the end of the ankle lever from the ankle joint center
			p3[0] =  self.d[3] * m.cos(FIXED_POINT_CONVERSION_DEG(DEG_TO_RAD(self.phi[0] + self.ticks_to_angle(self.ankleTicksAbsZeroed))));
			p3[1] =  self.d[3] * m.sin(FIXED_POINT_CONVERSION_DEG(DEG_TO_RAD(self.phi[0] + self.ticks_to_angle(self.ankleTicksAbsZeroed))));

			#print(("Left" if self.side == LEFT else "Right") +" p3 : " + str(p3))

			# x4 is the location where the strap crosses the center line between p2 and p3
			x4[0] = (self.r[2] * p3[0] + self.r[3] * p2[0]) / (self.r[2] + self.r[3]);
			x4[1] = (self.r[2] * p3[1] + self.r[3] * p2[1]) / (self.r[2] + self.r[3]);

			#print(("Left" if self.side == LEFT else "Right") +" x4 : " + str(x4))

			# t32 is the tangent point on the ankle lever
			t32[0] = (m.pow(self.r[3] , 2) * (x4[0] - p3[0]) + self.r[3] * (x4[1] - p3[1]) * m.sqrt(m.pow( x4[0] - p3[0] , 2) + m.pow(x4[1] - p3[1] , 2) - m.pow(self.r[3] , 2))) / (m.pow(x4[0] - p3[0] , 2) + m.pow(x4[1] - p3[1] , 2)) + p3[0];
			t32[1] = (m.pow(self.r[3] , 2) * (x4[1] - p3[1]) - self.r[3] * (x4[0] - p3[0]) * m.sqrt(pow( x4[0] - p3[0] , 2) + m.pow(x4[1] - p3[1] , 2) - m.pow(self.r[3] , 2))) / (m.pow(x4[0] - p3[0] , 2) + m.pow(x4[1] - p3[1] , 2)) + p3[1];

			# this is the unit vector for the strap from t32 to x4
			strap_dir[0] = (x4[0] - t32[0]) / m.sqrt(pow(x4[0] - t32[0] , 2) + m.pow(x4[1] - t32[1], 2));
			strap_dir[1] = (x4[1] - t32[1]) / m.sqrt(pow(x4[0] - t32[0] , 2) + m.pow(x4[1] - t32[1], 2));
			
			#print( ("Left" if self.side == LEFT else "Right") +" strap_dir : " + str(strap_dir))

			# take the cross product of the ankle lever and the strap direction
			momentArmLever = t32[0] * strap_dir[1] - t32[1] * strap_dir[0];
			
			#print(("Left" if self.side == LEFT else "Right") + " momentArmLever : " + str(momentArmLever))
			
			if momentArmLever < 0 :
				
				print(("Left" if self.side == LEFT else "Right") + " ExoBoot :: ankle_torque_to_current : moment arm is negative setting it to a small positive value as it is likely at an extreme position")
				momentArmLever = 1  #set it to a small value
			
			# get the current based on the torque cmd and the system state
			current = (torque / momentArmLever * self.r[1] + self.motorJ * self.data_current[self.idx_motor_accl]+ self.motorB * self.data_current[self.idx_motor_vel]+ self.motorBs) / self.kt;
			#print(("Left" if self.side == LEFT else "Right") + " torque / momentArmLever * self.r[1] : " + str(torque / momentArmLever * self.r[1]))
			#print(("Left" if self.side == LEFT else "Right")+ " self.motorJ * self.data_current[self.idx_motor_accl] : " + str(self.motorJ * self.data_current[self.idx_motor_accl]))
			#print(("Left" if self.side == LEFT else "Right") + " self.motorB * self.data_current[self.idx_motor_vel] : " + str(self.motorB * self.data_current[self.idx_motor_vel]))
			#print(("Left" if self.side == LEFT else "Right") + " self.motorBs) / self.kt : " + str((self.motorBs) / self.kt))

			#print(("Left" if self.side == LEFT else "Right") + " current : " + str(current) + "\n\n\n")

		return current;
	
	def ticks_to_angle(self, ticks) :
		return ticks * TICKS_TO_ANGLE_COEFF

	def bits_to_accl (self, bitReading) :
		return bitReading * BIT_TO_ACCL_COEFF

	def bits_to_gyro (self, bitReading) :
		return bitReading * BIT_TO_GYRO_COEFF

	def define_position_gains (self, kp, ki) : # for impedance also take in the stiffness and damping
		self.positionKp	= kp;
		self.positionKi	= ki;
		
	def define_current_gains (self, kp, ki) : # for impedance also take in the stiffness and damping
		self.currentKp	= kp;
		self.currentKi	= ki;
		
	def define_impedance_gains (self, kp, ki, k, b)  : # for impedance also take in the stiffness and damping
		self.impedanceKp	= kp;
		self.impedanceKi	= ki;
		self.impedanceStiffness	= k;
		self.impedanceDamping	= b;
		
	
	def set_controller(self, controlMode) :
		self.mode = controlMode;	# store the mode so we can easily check what is set as the Dephy lib doesn't return anything to know the current mode
		setControlMode(self.devId, self.mode);  # change the control mode to the one requested,  THIS MUST BE DONE BEFORE THE GAINS ARE SENT TO THE EXO
		
		if self.mode == CTRL_NONE :
			self.current_cmd = None
			setGains(self.devId, 0, 0, 0, 0);  # clear the gains
		
		elif self.mode == CTRL_OPEN:
				self.current_cmd = None
				self.setExoVoltage(0);  # start the open mode at zero so there are no surprises
		
		elif self.mode == CTRL_POSITION:
				self.current_cmd = None
				setGains(self.devId, self.positionKp, self.positionKi, 0, 0);  # set the gains for the position control		
		
		elif self.mode == CTRL_CURRENT:
				self.current_cmd = 0
				self.set_exo_current(0);  # set the current to zero to avoid surprises
				setGains(self.devId, self.currentKp, self.currentKi, 0, 0);  # set the gains for the current control
		
		elif self.mode == CTRL_IMPEDANCE:
				self.current_cmd = None
				setGains(self.devId, self.impedanceStiffness, self.impedanceDamping, self.impedanceKp, self.impedanceKi);  # set the gains for the impedance controller
		
		else :
			self.current_cmd = None
			setGains(self.devId, 0, 0, 0, 0);  # clear the gains

	#
	#
	#
	def set_exo_torque (self, torque, units = 'Nm') : 
		if (self.mode != CTRL_CURRENT) :
			self.set_controller (CTRL_CURRENT);
		
		if units == 'Nm' :
			self.set_exo_current(A_TO_MA(self.ankle_torque_to_current(NM_TO_NMM(torque))))
		elif units == 'Nmm' or units == 'mNm' :
			self.set_exo_current(A_TO_MA(self.ankle_torque_to_current(torque)))
		else :
			print('EXO_DEFS :: set_exo_torque : UNITS NOT PROPERLY SET')

	#
	#	Sends the current command accounting for side, positive is plantar flexion
	#
	def set_exo_current(self, currentCommand) :
		if (abs(currentCommand) < CURRENT_LIMIT) :
			self.current_cmd = currentCommand
			setMotorCurrent(self.devId, self.side*currentCommand);  # set the current on the exo
		
		else :
			print("exoBoot :: set_exo_current : CURRENT TOO HIGH, requested " + str(currentCommand) + " mA")
			self.current_cmd = CURRENT_LIMIT
			setMotorCurrent(self.devId, self.side*CURRENT_LIMIT);  # set the current on the exo
			# cout << "exoBoot :: set_exo_current : CURRENT TOO HIGH, requested " << currentCommand << " mA" << endl;

	#
	# Sends the motor position command accounting for side, takes in the fixed point motor angle without side adjustment
	#
	def set_exo_position(self, positionCommand) :
		#TODO: check the exo position command works
		self.current_cmd = None
		setPosition(self.devId, self.motorTicksOffset + self.side * positionCommand);  # the command should be absolute but the system takes in the relative position
		# this_thread::sleep_for(CMD_DELAY);  # delay to give time to communicate with the exo
	
	def go_slack (self) :
		distFromSlackPosition = self.motorSlackPosition - self.motorTicksRaw;

		# If the current position or the motor is more than one step away from the slack position set the desired position to a step from the current position.
		if (abs(distFromSlackPosition) > SLACK_STEP_SIZE) :
			if (self.mode != CTRL_POSITION) :
				self.set_controller (CTRL_POSITION);
			
			self.set_exo_position(self.motorTicksRaw + ((distFromSlackPosition > 0) - (distFromSlackPosition < 0)) * SLACK_STEP_SIZE);  # Commented out till output tested.
		
		else :  # if we are within a step just make it passive.
			self.set_controller (CTRL_NONE);
			
	def zero_current (self) :
		if (self.mode != CTRL_CURRENT) :
			self.set_controller (CTRL_CURRENT);
		self.set_exo_current(0)
		
		
	def init_collins_profile(self, mass = None, ramp_start_percent_gait = None, onset_percent_gait = None, peak_percent_gait = None, stop_percent_gait = None,  onset_torque = None, normalized_peak_torque = None) :
		# average values from the zhang/collins optimization paper.
		# t0 = 0;
		# t1 = 27.1;
		# t2 = 50.4;
		# t3 = 62.7;
		# ts = 2;

		# peakTorqueNormalized = 0.20; # 0.76; # Using a smaller value due to Dephy Exo Limit.
		
		if (mass != None) :
			self.user_mass = mass # kg
		if (ramp_start_percent_gait != None) :
			self.t0 = ramp_start_percent_gait
		if (onset_percent_gait != None) :
			self.t1 = onset_percent_gait
		if (peak_percent_gait != None) :
			self.t2 = peak_percent_gait
		if (stop_percent_gait != None) :
			self.t3 = stop_percent_gait
		if (onset_torque != None) :
			self.ts = onset_torque
		if (normalized_peak_torque != None) :
			self.peak_torque_normalized = normalized_peak_torque
		
			
		if (self.user_mass != -1 and self.t0  != -1, self.t1  != -1 and self.t2  != -1 and self.t3  != -1 and self.ts  != -1and self.peak_torque_normalized  != -1) :
			
			self.tp = self.user_mass * self.peak_torque_normalized;
			
			self.a1 = (2 *(self.tp - self.ts))/m.pow((self.t1 - self.t2),3);
			self.b1 = -((3 *(self.t1 + self.t2) *(self.tp - self.ts)) / m.pow((self.t1 - self.t2),3));
			self.c1 = (6* self.t1 * self.t2 * (self.tp - self.ts))/ m.pow((self.t1 - self.t2),3);
			self.d1 = -((-m.pow(self.t1, 3) *self.tp + 3 * m.pow(self.t1, 2)* self.t2 * self.tp - 3 * self.t1 * m.pow(self.t2,2) * self.ts + m.pow(self.t2,3) * self.ts)/ m.pow((self.t1 - self.t2),3));

			
			self.a2 = -((self.tp - self.ts)/(2* m.pow((self.t2 - self.t3),3)));
			self.b2 = (3 *self.t3 *(self.tp - self.ts))/(2 * m.pow((self.t2 - self.t3),3));
			self.c2 = (3 *(m.pow(self.t2,2) - 2 *self.t2 *self.t3) * (self.tp - self.ts))/(2* m.pow((self.t2 - self.t3),3));
			self.d2 = -((3 * m.pow(self.t2,2) * self.t3 * self.tp - 6 * self.t2 * m.pow(self.t3, 2) * self.tp + 2 * m.pow(self.t3,3) * self.tp - 2 * m.pow(self.t2,3) * self.ts + 3 * m.pow(self.t2, 2) * self.t3 * self.ts)/(2 * m.pow((self.t2 - self.t3), 3)));
			
		else :
			print('ExoBoot :: init_collins_profile : one of the parameters is not set' + \
				'\n user_mass : ' + str(self.user_mass) + \
				'\n ramp_start_percent_gait : ' + str(self.t0) + \
				'\n onset_percent_gait : ' + str(self.t1) + \
				'\n peak_percent_gait : ' + str (self.t2) + \
				'\n stop_percent_gait : ' + str (self.t3) + \
				'\n onset_torque : ' + str (self.ts) + \
				'\n normalized_peak_torque : ' + str (self.peak_torque_normalized))
		
		
	def run_collins_profile(self, external_read = False) : 
		# update data
		if not external_read :
			self.read_data()
		#print('exoBoot :: run_collins_profile : side : ' + ('LEFT' if self.side == LEFT else 'RIGHT')	+ ' : percent_gait : ' + str(self.percent_gait))
		
		if (self.percent_gait != -1) : 
			if ((self.percent_gait <= self.t1)  and  (self.t0 <= self.percent_gait)) : # torque ramp to ts at t1
				# 1 cout << "exoBoot :: runCollinsProfile : In t1 region" << endl;
				if (self.mode != CTRL_CURRENT) :
					self.set_controller (CTRL_CURRENT);
				
				tau = self.ts / (self.t1 - self.t0) * self.percent_gait - self.ts/(self.t1 - self.t0) * self.t0;
				# 1 cout << "exoBoot :: runCollinsProfile : tau = " << tau << endl;
				self.set_exo_current(max(NO_SLACK_CURRENT, A_TO_MA(self.ankle_torque_to_current(NM_TO_NMM(tau))))); #Commented out till output tested.
				#print ('exoBoot :: run_collins_profile : side : ' + ('LEFT' if self.side == LEFT else 'RIGHT')	+ '  T1 Region : tau : ' + str(tau) )
			
			elif (self.percent_gait <= self.t2) : # the rising spline
				# 1 cout << "exoBoot :: runCollinsProfile : In t2 region" << endl;
				if (self.mode != CTRL_CURRENT) :
					self.set_controller (CTRL_CURRENT);
				
				tau = self.a1 * m.pow(self.percent_gait,3) + self.b1 * m.pow(self.percent_gait,2) + self.c1 * self.percent_gait + self.d1;
				# 1 cout << "exoBoot :: runCollinsProfile : tau = " << tau << endl;
				self.set_exo_current(A_TO_MA(self.ankle_torque_to_current(NM_TO_NMM(tau)))); #Commented out till output tested.
				# print ('exoBoot :: run_collins_profile : side : ' + ('LEFT' if self.side == LEFT else 'RIGHT')	+ '  T2 Region : tau : ' + str(tau) )
				
			elif (self.percent_gait <= self.t3) : # the falling spline
				# 1 cout << "exoBoot :: runCollinsProfile : In t3 region" << endl;
				if (self.mode != CTRL_CURRENT) :
					set_controller (CTRL_CURRENT);
				
				tau = self.a2 * m.pow(self.percent_gait,3) + self.b2 * m.pow(self.percent_gait,2) + self.c2 * self.percent_gait + self.d2;
				# 1 cout << "exoBoot :: runCollinsProfile : tau = " << tau << endl;
				self.set_exo_current(A_TO_MA(self.ankle_torque_to_current(NM_TO_NMM(tau)))); #Commented out till output tested.
				#print ('exoBoot :: run_collins_profile : side : ' + ('LEFT' if self.side == LEFT else 'RIGHT')	+ '  T3 Region : tau : ' + str(tau) )
			
			else : # go to the slack position if we aren't commanding a specific value
				tau = 0;
				# 1 cout << "exoBoot :: runCollinsProfile : Going Slack" << endl;
				# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
				# !! if using go slack pick small proportional control !!
				# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
				# self.go_slack();
				self.set_exo_current(NO_SLACK_CURRENT);  # just enough to keep a small tension in the cable
				#print ('exoBoot :: run_collins_profile : side : ' + ('LEFT' if self.side == LEFT else 'RIGHT')	+ '  T4 Region : tau : ' + str(tau) )
			#print ('exoBoot :: run_collins_profile : side : ' + ('LEFT' if self.side == LEFT else 'RIGHT')	+ '  current_cmd (mA) : ' + str(A_TO_MA(self.ankle_torque_to_current(NM_TO_NMM(tau)))))
	
	def percent_gait_calc(self) :
		if (-1 != self.expected_duration)  : # if the expected duration is set calculate the percent gait
			self.percent_gait = 100 * (self.data_current[self.idx_time] - self.heelstrike_timestamp_current) / self.expected_duration;
				
		if (100 < self.percent_gait) : # if it has gone past 100 just hold 100
			self.percent_gait = 100;
		#print ('exoBoot :: percent_gait_calc : side : ' + ('LEFT' if self.side == LEFT else 'RIGHT')	+ '  percent_gait : ' + str(self.percent_gait) )
	
	def update_expected_duration(self) : 
		# TODO : In addition to checking that the step time is within a range, also check the the time it is armed is within the typical time.  Common errors occur from short spikes in acceleration that can have a close frequency.
		
		step_time = self.heelstrike_timestamp_current - self.heelstrike_timestamp_previous
		# armed_time = 0
		# if self.armed_timestamp != -1 :
			# armed_time = self.heelstrike_timestamp_current - self.armed_timestamp
		if (-1 == self.heelstrike_timestamp_previous) : # if it is the first time running just record the timestamp
			self.heelstrike_timestamp_previous = self.heelstrike_timestamp_current;
			return;
		if  (-1 in self.past_gait_times) : # if all the values haven't been replaced
			self.past_gait_times.insert(0, step_time);  # insert the new value at the beginning
			self.past_gait_times.pop(); # remove the last value
		elif ((step_time <= 1.75 * max(self.past_gait_times)) and (step_time >= 0.25 * min(self.past_gait_times))) : # and (armed_time > ARMED_DURATION_PERCENT * self.expected_duration)): # a better check can be used.  If the person hasn't stopped or the step is good update the vector.  
		# !!!THE ARMED TIME CHECK STILL NEEDS TO BE TESTED!!!
			self.past_gait_times.insert(0, step_time);  # insert the new value at the beginning
			self.past_gait_times.pop(); # remove the last value
			# TODO: Add rate limiter for change in expected duration so it can't make big jumps
			self.expected_duration = sum(self.past_gait_times)/len(self.past_gait_times);  # Average to the nearest ms

		#print ('exoBoot :: update_expected_duration : side : ' + ('LEFT' if self.side == LEFT else 'RIGHT')	+ '  expected_duration : ' + str(self.expected_duration) )
		
	def clear_gait_estimate(self) :
		self.past_gait_times = [-1] * NUM_GAIT_TIMES_TO_AVERAGE	# store the most recent gait times
		self.expected_duration = -1	# current estimated gait duration
		
	
	def check_for_heelstrike(self) :
		# the trigger on the inversion of the leg is one method.
		# can also use spikes in acceleration (X seems to be best candidate but may not work well for slower gaits with smaller impacts)
		# Other candidates also possible.
		triggered = False
		armed_time = 0
		if self.armed_timestamp != -1 :
			armed_time = self.data_current[self.idx_time] - self.armed_timestamp
		if ((not self.heelstrike_armed) and self.data_current[self.idx_gyro_z] >= self.segmentation_arm_threashold) :
			self.heelstrike_armed = True
			self.armed_timestamp = self.data_current[self.idx_time]
		if (self.heelstrike_armed and (self.data_current[self.idx_gyro_z] <= self.segmentation_trigger_threashold) ) :
			self.heelstrike_armed = False
			self.armed_timestamp = -1
			if  (armed_time > ARMED_DURATION_PERCENT/100 * self.expected_duration) :
				triggered = True
			
			
		self.segmentation_trigger = triggered
		#print ('exoBoot :: check_for_heelstrike : side : ' + ('LEFT' if self.side == LEFT else 'RIGHT')	+ ' : GyroZ : ' + str(self.data_current[self.idx_gyro_z] ) + ' : heelstrike_armed : ' + str(self.heelstrike_armed) + ' : triggered : ' + ('True' if triggered else 'False'))
	
	def __del__(self):
		# TODO: make where it can take single or multiple values
		#if self.should_log :
		if self.data_file != None :
			self.data_file.close()
		
		if self.use_tactor :
			del self.tactor
		fxStopStreaming(self.devId)
		closePort(self.idx)
		
	