#!/usr/bin/env python3

import sys, copy, rospy

from geometry_msgs.msg import Twist, Point, Pose
from std_msgs.msg import ColorRGBA, Float32

from spherov2 import scanner
from spherov2.sphero_edu import SpheroEduAPI
from spherov2.types import Color


msg = """

---------------------------------------------
Your input value is beyond the defined range!
It will not be changed. 
---------------------------------------------


"""


## scanning for BLE devices to discover all UUID characteristics
def getToysList():

	toyNames = []
	toys = scanner.find_toys()	
	if not toys:	
		print(' ')	
		print('The Sphero-BOLTs were not found!')
		print(' ')
		sys.exit()
		
	else:	
		print(' ')
		print('Following Sphero-BOLTs are discovered: ')
		print(' ')

		for t in toys:
			toyNames.append(t.name)
			print(t.name)		
			
	return toys, toyNames
	
	
def connectToy(toys_list, toy_names):

	_index = 0
	
	print(' ')
	toy_name = input("Which Sphero-BOLT do you want to use? Please enter its name: ")
	print(' ')
	
	if toy_name in toy_names:
		_index = toy_names.index(toy_name)				
					
					
	print('The index of the desired sphero-BOLT is: ', _index)
			
	_toy = toys_list[_index]	
	print(' ') 
	print(f'Connect to your desired sphero-BOLT: ' + toys_list[_index].name)
	print(' ')

	return _toy


class BoltControl(object):


	## constructor. This method is called when an object is created from the class.
	def __init__(self, my_toy):

		## initializing the attributes and declaring a node'
		super(BoltControl, self).__init__()

		## declaring the node 'twist_bolt_control'
		rospy.init_node('twist_bolt_control', anonymous=True, disable_signals=True)   
		
		self.position_pub = rospy.Publisher('/bolt/bolt_position', Point, queue_size=10)
		self.pose_pub = rospy.Publisher('/bolt/bolt_pose', Pose, queue_size=10)		
		
		rospy.Subscriber('/bolt/cmd_duration', Float32, self.duration_callback)	
		rospy.Subscriber('/bolt/cmd_color', ColorRGBA, self.color_callback)					
				

		# Initialise the messages of the robot
		
		self.speed = 0
		self.angular = 0
		
		self.duration = 0.5
		rospy.loginfo("start value: duration = %d  ", self.duration)		
		
		self.color_r = 255
		self.color_g = 128
		self.color_b = 128		
		rospy.loginfo("start value: color = (%s)", str(self.color_r) + "," + str(self.color_g) + "," + str(self.color_b))		
		

		self.position_msg = Point()
		self.position_msg.x = 0
		self.position_msg.y = 0
		self.position_msg.z = 0
		
		self.pose_msg = Pose()
		self.pose_msg.position.x = 0
		self.pose_msg.position.y = 0
		self.pose_msg.position.z = 0	
		self.pose_msg.orientation.x = 0
		self.pose_msg.orientation.y = 0
		self.pose_msg.orientation.z = 0
		self.pose_msg.orientation.w = 0			
				
		self.twist_msg = Twist()		
		self.spheroBolt_start(my_toy)	
		


	def spheroBolt_start(self, my_toy):

		my_toy.set_main_led(Color(r=self.color_r, g=self.color_g, b=self.color_b))   
		

	
	def getSpheroBoltPosiotion(self, _my_toy):
	
		
		self.position_msg.x = round(_my_toy.get_location()['x'],4)
		self.position_msg.y = round(_my_toy.get_location()['y'],4)
		self.position_msg.z = 0
		
		
		return self.position_msg	
		

	def getSpheroBoltPose(self, _my_toy):
	

		self.pose_msg.position.x = round(_my_toy.get_location()['x'],4)
		self.pose_msg.position.y = round(_my_toy.get_location()['y'],4)
		self.pose_msg.position.z = 0	
		self.pose_msg.orientation.x = round(_my_toy.get_orientation()['pitch'],4)
		self.pose_msg.orientation.y = round(_my_toy.get_orientation()['roll'],4)
		self.pose_msg.orientation.z = round(_my_toy.get_orientation()['yaw'],4)
		self.pose_msg.orientation.w = 0
		
		rospy.loginfo('current position: {},{} -- current orientation: {} '.format(self.pose_msg.position.x, self.pose_msg.position.y, self.pose_msg.orientation.z))
		
		return self.pose_msg			
		
	
			
	## subscribing on the topic '/bolt/cmd_vel'
	## When new messages are received, callback 'sphero_callback' is invoked with the message '_key_msg'.	
	def bolt_subscriber_publish(self, my_toy):	
		
		rospy.Subscriber('/bolt/cmd_vel', Twist, self.sphero_callback)			
					
		number = 0
		cycle_counts = 1300
		
		try:
			while(1):
				number = number + 1 			

				if (number == cycle_counts):
					self.stop_movement(my_toy)										
					break
				else:
					self.spheroBolt_execute_action(my_toy)
			
			rospy.spin()
			
		except KeyboardInterrupt:
			self.stop_movement(my_toy)
			rospy.loginfo("The program terminated with keyboard interrupt.")
			sys.exit(0)					



	## bolt is running ...		
	def spheroBolt_execute_action(self, my_toy):

		my_toy.set_main_led(Color(r=self.color_r, g=self.color_g, b=self.color_b))
			
		
		my_toy.set_speed(0) 			
		my_toy.roll( self.angular, self.speed, self.duration)
		
		self.position_pub.publish(self.getSpheroBoltPosiotion(my_toy))
		self.pose_pub.publish(self.getSpheroBoltPose(my_toy))	

	
	## setting the speed of the sphero-bolt from -255 to 255, where 0 is stopped.
	def stop_movement(self, my_toy):

		my_toy.set_speed(0)
		my_toy.strobe(Color(255, 0, 0), (3 / 15) * .5, 15)


	## When new Twist-messages are received, callback is invoked with the message '_key_msg'.
	def sphero_callback(self, _key_msg):  		
		
		self.twist_msg = _key_msg
		self.speed = int(round(self.twist_msg.linear.x))
		
		rospy.loginfo("execute the action: speed = %d  -- angular-speed: %d", self.speed, int(round(self.twist_msg.angular.z))%360)
		
		self.angular = self.angular + int(round(self.twist_msg.angular.z))
		

	## When new duration-messages are received, callback is invoked with the message '_duration_msg'.
	def duration_callback(self, _duration_msg):  		

		if (0.1 <_duration_msg.data < 9):	
			self.duration = _duration_msg.data
			rospy.loginfo("execute the action: duration = %f  ", self.duration)
		else:
			print(msg)
			
		
		
	## When new Color-messages are received, callback is invoked with the message '_color_msg'.
	def color_callback(self, _color_msg):  		
	
		if ((0<=int(_color_msg.r)<=255) and (0<=int(_color_msg.g)<=255) and (0<=int(_color_msg.b)<=255)):
			self.color_r = int(_color_msg.r)
			self.color_g = int(_color_msg.g)
			self.color_b = int(_color_msg.b)					
			#self.color_a = _color_msg.a			

			rospy.loginfo("execute the action: color = (%s)", str(self.color_r) + "," + str(self.color_g) + "," + str(self.color_b))
			
		else:
			print(msg)
			

if __name__== "__main__":

	## discovering and connecting a bolt
	toys, toyNames = getToysList()
	toy = connectToy(toys, toyNames)
	
	if toy is not None:
		with SpheroEduAPI(toy) as toy_:			
			## a new BoltControl-Class's instance is created.	
			new_toy = BoltControl(toy_)
			while not rospy.is_shutdown():
				new_toy.bolt_subscriber_publish(toy_)			
  
