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
# Make sure to put in numbers for the host and port.    These are just the default values so you can set them to what you commonly use.
# !!!

import socket
import time

HOST_TCP_HANDLER = <IP_ADDRESS_OF_THE_HOST>  # example keep the quotes :  '192.168.1.1'
PORT_TCP_HANDLER = <PORT_THE_HOST_IS_LISTENING_TO> # example: 67876



class TcpHandler :
	def __init__ (self, host = HOST_TCP_HANDLER, port = PORT_TCP_HANDLER, max_size = 1024, period = 3) :
		self.host = host 
		self.port = port
		self.max_size = max_size
		self.period = period
		self.send_msg = bytearray([0, 0, 0])
		self.recv_msg = bytearray([0, 0, 0, 0])
		self.last_send = time.monotonic()
		
		self.send_len = bytearray([0, 254])
		self.recv_len = bytearray([0, 0])
		self.update_vals
		
	def heartbeat(self) :
		cur_time = time.monotonic()
		updated = 0
		if (self.period < (cur_time - self.last_send)) :
			self.send_recv()
			self.last_send = cur_time
			updated = 1
			
		return updated
	
	
	def send_recv(self, send_msg =   None) :
		
		if (send_msg != None)  :
			self.send_msg = send_msg
		
		# check if it is a byte array and change it if it isn't
		if (not isinstance(self.send_msg, bytearray)) :
			self.send_msg = bytearray(self.send_msg)
		
		
		s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		s.connect((self.host, self.port))
		
		s.send(self.send_msg)
		self.recv_msg = s.recv(self.max_size)
		
		s.close
		
		print(int(self.last_send))
		print('sent : \t\t' + str(self.send_msg[0]) + '\t' + str(self.send_msg[1]) + '\t' + str(self.send_msg[2]))
		
		#print(self.send_msg)
		print('received : \t' + str(self.recv_msg[0]) + '\t' + str(self.recv_msg[1]) + '\t' + str(self.recv_msg[2]) + '\t' + str(self.recv_msg[3]))
		#print(bytearray(self.recv_msg))
		#print(self.recv_msg[2])
		print('\n')
		
		self.update_vals()
		self.last_send = time.monotonic()
		
	def update_vals(self) :
		self.send_len =len(self.send_msg)
		self.recv_len = len(self.recv_msg)
	




	