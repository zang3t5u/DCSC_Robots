import os
import sys
import time
import struct

#tos stuff
import Bot_NetMsg
from tinyos.message import *
from tinyos.message.Message import *
from tinyos.message.SerialPacket import *
from tinyos.packet.Serial import Serial

#ROS Stuff
import rospy
from std_msgs.msg import String
from dcsc_consensus.msg import bot_data_msg

#Total TimeInterval in millisecs for 1 bot in TDMA
t_interval = 250
NBots = 1

#Check if UART is free
send_queue_size = 50
uartBusy = False
recvUART = False	#If message recvd on UART, UART is locked
	

class Bot_Net:
	def callback(self, data):
		rospy.loginfo(rospy.get_caller_id() + "I heard from %d", data.botID)

	def __init__(self, motestring, Num_of_Bots, botID):

		global NBots, send_queue_size
		
		self.counter = 0
		self.total_msgs = 0.
		self.mif = MoteIF.MoteIF()
		self.tos_source = self.mif.addSource(motestring)
		self.mif.addListener(self, Bot_NetMsg.Bot_NetMsg)

		NBots = Num_of_Bots
		self.motestring = motestring
		self.botID = botID
		
		'''For Msg Communication
		self.send_queue = [Bot_NetMsg.Bot_NetMsg()]*send_queue_size
		self.next_msg_index = 0
		'''
		#Each bot Stores data of all bots
		self.recv_seqNo = [-1]*(NBots+1)	#Store last received seqNo. from Bots + centralComp to Discard duplicates
		self.discarded = [-1]*(NBots+1)		#Keep count of discarded messages from each source
		self.bot_data = []
		for i in range(NBots):
			self.bot_data.append([0]*10)
			self.bot_data[i][0] = i+1
		'''
		Index							Data
		-----							----
		0									ID
		1									X
		2									Y
		3									Theta
		4									VelX
		5									VelY
		6									VelTheta
		7									Leader X
		8									Leader Y
		9									last_update_time
		'''
		#ROS
		topic_name = 'talker'+str(botID)
		self.pub = rospy.Publisher(topic_name, bot_data_msg, queue_size=10)
		self.publish_data = [0]*(NBots+1)
		
		for i in range(Num_of_Bots):
			rospy.Subscriber('talker'+str(int(botID)), bot_data_msg, self.callback)
		#For TimeSync
		self.start = False
		print "Start clock?: ", self.start
		self.start_time = time.time()
		self.timeOffset = self.botID*t_interval
		self.last_recv_time = 0
		print "TDMA Offset: ", self.timeOffset

	def receive(self, src, msg):
		global recvUART
		recvUART = True  	
		#List of data to be returned
		abstracted_data = []
		if msg.get_amType() == Bot_NetMsg.AM_TYPE:
			self.last_recv_time = time.time() - self.start_time
			#Get Msg Source
			msg_source = msg.getAddr()
			m = Bot_NetMsg.Bot_NetMsg(msg.dataGet())
			#msg_source = m.get_send_ID()
			
			#If first message received from Central Comp, start timer
			if((not self.start) and msg_source==0):
				print "Starting Clock NOW!!!"
				self.start_time = self.last_recv_time + self.start_time - m.get_tx_timestamp()/1000.0
				print "at Time: ", self.start_time
				self.last_recv_time = 0
				self.recv_seqNo = [-1]*(NBots+1)
				self.discarded = [-1]*(NBots+1)
				self.start = True
			
			#Discard duplicate or old messages. 
			#If more than 2 messages discarded, accept message, To account for dropped messages during rollover from 255 to 0
			if((m.get_seqNo() < self.recv_seqNo[msg_source]) and (self.discarded[msg_source])<2):
				#print "Old ", m.get_seqNo()," from ", msg_source
				sys.stdout.flush()
				self.discarded[msg_source] += 1
				recvUART = False
				return
			else:
				self.recv_seqNo[msg_source] = (m.get_seqNo()+1)%256
				self.discarded[msg_source] = 0
			
			#Msg sent by Central Comp
			if(msg_source == 0):
				#Data in message describes the recipient
				msg_bot = m.get_recv_Rob_ID()
			#Msg sent by Bot
			else:
				#Data in message describes the sender
				msg_bot = msg_source
				
			#Extend list to include information
			abstracted_data.extend([float(m.get_tx_timestamp())])
			abstracted_data.extend([float(m.get_seqNo())])
			abstracted_data.extend([msg_bot])
			abstracted_data.extend([m.get_dataType()])
			abstracted_data.extend([m.get_data()])
			self.decode_data(msg_bot, m, msg_source)
			'''  			
			#Print information
			print "Received", msg
			sys.stdout.flush()
			'''
		#Return
		#Print information
		print "From ", msg_source, " Received ", abstracted_data, " at ", 1000*self.last_recv_time
		sys.stdout.flush()
		recvUART = False
		#self.bot_data[msg_bot].append(abstracted_data)
    
	def send_msg(self, send_ID, recv_Rob_ID, dataType, data):
    	
		global uartBusy, NBots, t_interval
		
		msg_was_sent = False
		
		if((self.start or (self.botID==0)) and (not uartBusy) and (not recvUART)):
			print "Sending packet ", self.counter
			smsg = Bot_NetMsg.Bot_NetMsg()
			smsg.set_seqNo(self.counter)
			smsg.set_send_ID(send_ID)
			smsg.set_recv_Rob_ID(recv_Rob_ID)
			smsg.set_dataType(dataType)
			if(len(data)<3):
				data.extend(0)
			smsg.set_data(data)
			
			#Wait for millisec timeslot 
			t = 1000*(time.time()-self.start_time) 
			cond = int((t%((NBots+1)*t_interval))/t_interval)
			if((not self.start) and (self.botID == 0)):
				print "Starting Clock NOW!!!"
				self.start_time = time.time()
				print "at Time: ", self.start_time
				self.recv_seqNo = [-1]*(NBots+1)
				self.discarded = [-1]*(NBots+1)
				self.start = True 
			
			while(cond != self.botID):
				t = 1000*(time.time()-self.start_time)
				cond = int((t%((NBots+1)*t_interval))/t_interval)
				#print t    
			smsg.set_tx_timestamp(t)
			
			#Send Packet twice to ensure delivery and minimize number of dropped packets
			packetCount = 0
			uartBusy = True
			#self.mif.sendMsg(self.tos_source, 0xFFFF, smsg.get_amType(), 0, smsg) 
			while(packetCount<2):
				#print "Sending packet ", self.counter," : Count ", packetCount
				while(recvUART):{} 
				self.mif.sendMsg(self.tos_source, 0xFFFF, smsg.get_amType(), 0, smsg) 
				packetCount+=1
				time.sleep(0.5)
			msg_was_sent = True
			uartBusy = False
			self.counter+=1
			self.counter%=256
		return msg_was_sent
    	
	def decode_data(self, msg_bot, m, msg_source):
		botIndex = msg_bot-1
		dataType = m.get_dataType()
		t = m.get_tx_timestamp()/1000.0
		data = m.get_data()
		seq = m.get_seqNo()
		
		#Check if received update is indeed new one and not hopped msg
		if(self.bot_data[botIndex][9] > t):		#Old update
			print "Old Update"
			return
		self.bot_data[botIndex][0] = botIndex+1
		'''
		POSE Data => dataType = 1
		Vel Data => dataType = 2
		Leader Data => dataType = 3
		'''
		self.publish_data[botIndex] = 1;

		#print " Storing for Robot", msg_bot, "at index: ", botIndex
		self.bot_data[botIndex][1 + 3*(dataType-1)] = data[0]
		self.bot_data[botIndex][2 + 3*(dataType-1)] = data[1]
		self.bot_data[botIndex][3 + 3*(dataType-1)] = data[2]
		self.bot_data[botIndex][9] = t		#Store last update timestamp
		#self.display_database()
    
	def display_database(self):
		if(self.start):
			print "+++++++++++++ Bot Database"
			for i in range(len(self.bot_data)):
				print "Robot ", self.bot_data[i][0], ": "
				print "    x: ", self.bot_data[i][1]
				print "    y: ", self.bot_data[i][2]
				print "theta: ", self.bot_data[i][3]
				print "-------"
		sys.stdout.flush()
'''
    def main_loop(self):
    	while 1:
    		time.sleep(1)
    		# send a message 1's per second
    		self.send_msg()
'''
'''
def main():
	if '-h' in sys.argv or len(sys.argv) < 2:
		print "Usage:", sys.argv[0], "sf@localhost:9002", "adc_rev_volt <Volt>" 
		sys.exit()
		
	dl = Bot_Net(sys.argv[1], float(sys.argv[2]))
	dl.main_loop()  # don't expect this to return...

if __name__ == "__main__":
	try:
		main()
	except KeyboardInterrupt:
		pass
'''
