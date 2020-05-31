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


#
# Based on  https://learn.sparkfun.com/tutorials/raspberry-pi-spi-and-i2c-tutorial/all
# for use with the TLC2543 ADC
#

import time
import spidev
import os.path

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

from datetime import datetime

# TLC2543 data input settings datasheet Table 2
# 4 MSB D7-4

# pin address is just the pin number

# test voltage
tlc_test_v_half = 11
tlc_test_v_0 = 12
tlc_test_v_full = 13
tlc_powerdown = 14

# data length bits, D3-2
tlc_8_bit = 1
tlc_12_bit = 0
tlc_16_bit = 3

# data order, bit D1
tlc_msb_first = 0
tlc_lsb_first = 1

# polarity, bit D0
tlc_unipolar = 0
tlc_bipolar = 1

# this part you choose to use, you can define it for each communication but if you are always using the same settings this is easier
# to use it you can do (AIN# << 4 | tlc_settings)
tlc_settings =   tlc_16_bit << 2 | tlc_msb_first << 1 | tlc_unipolar

MAX_FILE_SIZE = 500 * 10**6 # 500 MB

class TLC2543:
	def __init__(self, bus = 0 , device = 0, max_speed_hz = 1000000, lsbfirst = False, delay_usec = 10, should_log = False, sync_pin = 40, mode = "BOARD"):
	# the Pi4 can go up to max_speed_hz = 4000000
	# the jetson nano can't handle that 2000000 works
	#	I will use 1000000 as the default to be safe
		# We only have SPI bus 0 available to us on the Pi
		self.bus = bus
		
		#Device is the chip select pin. Set to 0 or 1, depending on the connections
		self.device = device
		
		# Enable SPI
		GPIO.setwarnings(False)
		self.spi = spidev.SpiDev()

		# Open a connection to a specific bus and device (chip select pin)
		self.spi.open(bus, device)

		# Set SPI speed and mode
		# this should match your settings for the TLC
		# TLC  clk 4.1 MHz !!! that was very wrong at least when looking at the scope it needs to be 50kHz otherwise it is real wavy but this could be due to the capacitance in the scopes.
		self.spi.max_speed_hz = max_speed_hz 

		# mode (CPOL << 1 | CPHA) 
		# Polarity if clock is normally low it is polarity 0 and the first event will be a rising edge
		# If it is normally high and the polarity is 1 and the first event will be a falling edge
		# Phase if the sample is taken on the first event then the phase is 0
		# If the sample is taken on the second event then the phase is 1
		# for the TLC CPOL = 0 and CPHA = 0
		self.spi.mode = 0
		self.spi.lsbfirst = lsbfirst 

		# I am not currently using EOC pin in which case we need to make sure we have enough of a break before we read so the conversion can complete 10us should be ok from data sheet operating characteristics
		self.delay_usec = delay_usec
		self.should_log = should_log
		
		self.mode = GPIO.getmode()
		if self.mode == None :  # if it is not already set set it
			self.mode = GPIO.BOARD if mode == "BOARD" else GPIO.BCM
			GPIO.setmode(self.mode)
		
		self.sync_pin = sync_pin
		
		#self.channels = channels

		# we are using 12 bit mode may want to switch to 16 if this gets goofy.  May need to shift the sent data by another 4 bits
		self.spi.bits_per_word = 8
		
		self.settings =   tlc_16_bit << 2 | self.spi.lsbfirst << 1 | tlc_unipolar
		
		self.current_milli_time = lambda: int(round(time.time() * 1000))
		
		self.start_time = self.current_milli_time()
		if self.should_log :
			self.file_base = "";
			self.file_extension = "";
			self.data_filename = "";
			
			
				
			self.data_file = self.log_data_init()
		
		
	def log_data_init (self, channels = [0, 1, 2, 3, 4, 5, 6, 7]):
		
		start_time = datetime.now()
		time_str = start_time.strftime("%Y_%m_%d_%Hh%Mm%Ss")
		
		
		# you need to initilize these before calling these
		self.file_base = time_str + "_adc"
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
		
		labels = "time, sync, " 
		for channel in channels :
			labels += "ADC" + str(channel) + ", "
			
		# labels = "Time, sync, " 
		# for channel in channels :
			# labels += "ADC" + str(channel) + ", "
		data_file.write(labels)
		data_file.write("\n ")
		
		return data_file
		
		# file_base = "test_data"
		# file_extension = ".csv"
		
		# self.data_filename = file_base + file_extension
		# i = 0
		# while os.path.exists(self.data_filename):
			# i +=1
			# self.data_filename =  file_base + "_"+ str(i) + file_extension
				
		# self.data_file = open(self.data_filename, 'a')
		
		# labels = "Time, " 
		# for channel in channels :
			# labels += "ADC" + str(channel) + ", "
		# self.data_file.write(labels)
		# self.data_file.write("\n ")
		
		
	def get_free_filename(self, base, extension) :
		data_filename = base + extension
		i = 0
		while os.path.exists(data_filename):
			i +=1
			data_filename =  base + "_"+ str(i) + extension
		return data_filename
	
	def log(self, values, channels = [0, 1, 2, 3, 4, 5, 6, 7]):
		if self.should_log :
			# check file size
			if os.path.getsize(self.data_filename) > MAX_FILE_SIZE :
				self.data_file.close()
				self.data_filename = self.get_free_filename(self.file_base, self.file_extension)
				self.data_file = open(self.data_filename, 'a')
			
				labels = "time, sync, " 
				for channel in channels :
					labels += "ADC" + str(channel) + ", "
				
				# labels_csv = "";
				# for label in self.labels_current :
					# labels_csv += label + ", "
				self.data_file.write(labels)
				self.data_file.write("\n ")
			values = [time.monotonic(), GPIO.input(self.sync_pin)] + values
			for i in range(0, len(values)):
				self.data_file.write(str(values[i]))
				self.data_file.write(",")
			self.data_file.write("\n ")
		else :
			print("TLC2543 :: log(self) : \n\tYOU WANTED TO LOG VALUES BUT LOGGING DATA WAS NOT SELECTED")
		
		# if self.log_data :
			# for i in range(0, len(values)):
				# self.data_file.write(str(values[i]))
				# self.data_file.write(",")
			# self.data_file.write("\n ")
		# else :
			# print("TLC2543 :: log_data(self, values) : \n\tYOU WANTED TO LOG VALUES BUT LOGGING DATA WAS NOT SELECTED")
		
	def read_channel(self, channel):
		#request the channel 
		readList = [channel << 4 | self.settings, 0]
#		print("sending : ")
#		print(readList)
		adc_values = self.spi.xfer2(readList)
		# read the results
		adc_values = self.spi.xfer2(readList)
#		print("receiving : ")
#		print(adc_values)
		reading = (adc_values[0]<<4 | adc_values[1] >> 4) 
		return reading
		
	def read_channels	(self, channels = [0, 1, 2, 3, 4, 5, 6, 7]):
		
		
		channel_prev = None;
		voltage_bin = [None] * len(channels)
		#channels.append(0) # add an additional data transfer so we can read the last result.
		
		# print(str( channels ) )
		i = 0
		for channel in (channels + [0]):
			#print("reading channel : " + str(channel))
			readList = [channel << 4 | tlc_settings, 0]
	#		print("sending : ")
	#		print(readList)
			adc_values = self.spi.xfer2(readList)
			
			#print("ADC return : " + str(adc_values))
	#		print("receiving : ")
	#		print(adc_values)
			# The ADC returns the value from the channel that was sent the previous time.
			if (channel_prev != None):
				voltage_bin[i] = (adc_values[0]<<4 | adc_values[1] >> 4) 
				i += 1 
			channel_prev = channel	
			
		return voltage_bin

	def convert_to_v(self, reading):
		voltage = reading / 4095 *5
		return voltage
		
	def __del__(self):
		# TODO: make where it can take single or multiple values
		if self.should_log:
			self.data_file.close()
		self.spi.close()
		
		
		
		