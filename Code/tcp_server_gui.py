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

"""
This script is used to create a GUI to interface with the Exo boot's study_code.py
The arrows must currently be used to for the final change to the study, trial, and mass
The start trial and treadmill stop buttons will turn green when the study code is ready for them
The host IP address in the study code should be changed to point at whatever computer is running this script
This can be the loopback IP address if you are running it on the same computer that is communicating with the boots but if you do this X11 forwarding should be setup.
"""

# !!!
# Make sure to change the port to match what is in the study_code.py
# !!!


from tkinter import *
import numpy as np
import socket
import time

DELAY_MS = 100

HOST = '127.0.0.1'
PORT = <PORT_THE_HOST_IS_LISTENING_TO> # example: 67876

STUDY_IDX = 0
TRIAL_IDX = 1
MSG_IDX = 2
USER_MASS_IDX = MSG_IDX + 1

MSG_STOP_PROGRAM = 255
MSG_STOP_TRIAL = 254
MSG_NOTHING_TO_DO = 253
MSG_TREADMILL_STOPPED = 252
MSG_START_TRIAL = 250

MSG_TRIAL_EXITED = 251
MSG_READY_FOR_TRIAL = 254

# CREATE RESPONSE CUE
def ready_for_click (widgit) :
	widgit.configure(bg="light green", fg="black")
def not_ready_for_click (widgit) :
	widgit.configure(bg="light gray", fg="black")
	
def alternate_colors(btn1,btn2) :
	ready_for_click(btn2)
	not_ready_for_click(btn1)
	
# CREATE TCP HANDLER
class tcpServer :
	def __init__ (self, host = HOST, port = PORT, max_size = 1024, sock = None) :
		self.host = host
		self.port = port
		self.max_size = max_size
		self.recv_msg = bytearray([0, 0, 0])		#study, trial, msg
		self.send_msg = bytearray([0, 0, MSG_NOTHING_TO_DO, 1]) #study, trial, msg, user mass
		#self.last_send = time.monotonic()
		
		self.study = 0
		self.trial = 0
		self.msg_byte = MSG_NOTHING_TO_DO
		self.user_mass = 0
		
		
		self.recv_len = bytearray([0, 254])
		self.send_len = bytearray([0, 0])
		
		if sock is None:
			self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		else:
			self.sock = sock

		# bind the socket to a public host, and a port
		self.sock.bind((socket.gethostname(), self.port))
		# become a server socket
		self.sock.listen(5)
		
		self.sock.setblocking(False) # set to be non blocking accept will return an error
		

def set_send_msg(tcp_server, study = None, trial = None, message = MSG_NOTHING_TO_DO, user_mass = None) :
	if study != None :
		tcp_server.send_msg[STUDY_IDX] = study
	if trial != None:
		tcp_server.send_msg[TRIAL_IDX] = trial
	if message != MSG_NOTHING_TO_DO :
		tcp_server.send_msg[MSG_IDX] = message
	if user_mass != None :
		tcp_server.send_msg[USER_MASS_IDX] = user_mass
	
	# check if it is a byte array and change it if it isn't
	if (not isinstance(tcp_server.send_msg, bytearray)) :
		tcp_server.send_msg = bytearray(self.send_msg)
	
def tcp_handler(tcp_server, treadmill_auto_stop) :
	try:
		
		
		(client_socket, address) = tcp_server.sock.accept()
		tcp_server.recv_msg = client_socket.recv(tcp_server.max_size)
		print(time.monotonic())
		print('\nreceived : \t' + str(tcp_server.recv_msg[0]) + '\t' + str(tcp_server.recv_msg[1]) + '\t' + str(tcp_server.recv_msg[2]))
		
		# print(treadmill_auto_stop)
		
		if tcp_server.recv_msg[MSG_IDX] == MSG_TRIAL_EXITED and treadmill_auto_stop :
			tcp_server.send_msg[MSG_IDX] = MSG_TREADMILL_STOPPED
			print('Overwrote whatever the message was with a treadmill stopped')
		
		print('sent : \t\t' + str(tcp_server.send_msg[0]) + '\t' + str(tcp_server.send_msg[1]) + '\t' + str(tcp_server.send_msg[2]) + '\t' + str(tcp_server.send_msg[3]))
		client_socket.send(tcp_server.send_msg)
		
		set_send_msg(tcp_server, message = MSG_NOTHING_TO_DO) # reset the message as nothing to do
	except BlockingIOError: 
		# no connection
		pass
		
	except:
		print ("Unexpected error:", sys.exc_info()[0] )
		
	
	
	return tcp_server.recv_msg[MSG_IDX]




window = Tk()
		
# this is called after 
def after_function (tcp_server, btn_start_trial, btn_treadmill_stopped, treadmill_auto_bool) :
	tcp_handler(tcp_server,treadmill_auto_bool.get())
	
	if tcp_server.recv_msg[MSG_IDX] == MSG_TRIAL_EXITED :
		ready_for_click(btn_treadmill_stopped)
	if tcp_server.send_msg[MSG_IDX] == MSG_TREADMILL_STOPPED :
		not_ready_for_click(btn_treadmill_stopped)
	
	if tcp_server.recv_msg[MSG_IDX] == MSG_READY_FOR_TRIAL :
		ready_for_click(btn_start_trial)
	else :
		not_ready_for_click(btn_start_trial)
	
	
	window.after(DELAY_MS, after_function, tcp_server, btn_start_trial, btn_treadmill_stopped, treadmill_auto_bool)
	
	


window.title("Exo TCP Server")

tcp_server = tcpServer()

font_size = 25
font_style = 'Arial'
#window.geometry('850x275')


study = StringVar()
lbl_study = Label(window, text="Study", font = (font_style, font_size))
lbl_study.grid(column=0, row=0)
box_study = Spinbox(window, from_=0, to=255, width=5, textvariable=study, font = (font_style, font_size), command = lambda: set_send_msg(tcp_server, study = int(study.get())))
box_study.grid(column=0, row=1)

trial = StringVar()
lbl_trial = Label(window, text="Trial", font = (font_style, font_size))
lbl_trial.grid(column=1, row=0)
box_trial = Spinbox(window, from_=0, to=255, width=5, textvariable=trial,  font = (font_style, font_size), command = lambda: set_send_msg(tcp_server, trial = int(trial.get())))
box_trial.grid(column=1, row=1)

user_mass= StringVar()
lbl_user_mass = Label(window, text="User Mass", font = (font_style, font_size))
lbl_user_mass.grid(column=2, row=0)
box_user_mass = Spinbox(window, from_=0, to=255, width=5, textvariable= user_mass, font = (font_style, font_size), command = lambda: set_send_msg(tcp_server, user_mass = int(user_mass.get())))
box_user_mass.grid(column=2, row=1)

lbl_instructions = Label(window, text="For study, trial,\nand mass the\nvalues are not\nset till the\narrows are uses", font = (font_style, int(font_size/2)))
lbl_instructions.grid(column=0, row=2)

btn_start_trial = Button(window, text="Start Trial", font = (font_style, font_size), bg = 'light gray', command = lambda: set_send_msg(tcp_server, message = MSG_START_TRIAL))
btn_start_trial.grid(column=3, row=1)

btn_treadmill_stopped = Button(window, text="Treadmill Stopped", font = (font_style, font_size), bg = 'light gray', command = lambda: set_send_msg(tcp_server, message = MSG_TREADMILL_STOPPED))
btn_treadmill_stopped.grid(column=3, row=2)

# btn_start_trial.configure(command = lambda:  alternate_colors(btn_start_trial, btn_treadmill_stopped))
# btn_treadmill_stopped.configure(command = lambda:  alternate_colors(btn_treadmill_stopped, btn_start_trial))

treadmill_auto_bool = BooleanVar()
treadmill_auto_bool.set(True) #set check state
chk_treadmill_auto = Checkbutton(window, text='Treadmill Auto Stop', var=treadmill_auto_bool, font = (font_style, font_size))
chk_treadmill_auto.grid(column = 3, row=3)

btn_stop_trial = Button(window, text="Stop Trial", bg="red", fg="black", font = (font_style, int(font_size * 1.25)),command = lambda: set_send_msg(tcp_server, message = MSG_STOP_TRIAL))
btn_stop_trial.grid(column=2, row=2)

btn_stop_program = Button(window, text="Stop Program", font = (font_style, font_size), bg = 'light gray', command = lambda: set_send_msg(tcp_server, message = MSG_STOP_PROGRAM))
btn_stop_program.grid(column=2, row=3)


window.after(DELAY_MS, after_function, tcp_server, btn_start_trial, btn_treadmill_stopped, treadmill_auto_bool)
# use window.after(delay_ms, function = callback, *args)  Can't find the documentation for it but the internet seems to think it is a thing.
window.mainloop()