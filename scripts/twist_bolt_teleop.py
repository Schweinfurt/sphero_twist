#!/usr/bin/env python3

import sys, select, os
import tty, termios
import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from std_msgs.msg import Float32



msg_in = """

Input arguments: filename duration(Valid values: 2~8) color-r(Valid values: 0~255) color-g(Valid values: 0~255) color-b(Valid values: 0~255)

---- for example ----

rosrun sphero_twist twist_bolt_teleop.py 3 255 0 255
"""

msg = """
Control your Sphero-Bolt!
---------------------------
Moving around:
        w
   a    s    d
	

space key or p : force stop

CTRL-C to quit
"""

e = """
Failed
"""


# reading input-character from keyboard
def getKey():

	tty.setraw(sys.stdin.fileno())
	rlist, _, _ = select.select([sys.stdin], [], [], 0.1)

	if rlist:
		key = sys.stdin.read(1)
	else:
		key = ''

	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key
		
	
def getDurationAndColor(_inputValue):

	#print(f"Arguments of the script : {_inputValue[0:]}")	
	durationValue_ = float(_inputValue[0])
	color_r_ = float(_inputValue[1])
	color_g_ = float(_inputValue[2])
	color_b_ = float(_inputValue[3])					
	color_a_ = 1.0		
	
	return durationValue_, color_r_, color_g_, color_b_, color_a_


if __name__=="__main__":

	settings = termios.tcgetattr(sys.stdin)

	# declaring the node 'twist_bolt_teleop'
	rospy.init_node('twist_bolt_teleop')
	
	# checking input parameters	
	if len(sys.argv) != 5:
		print(msg_in)
		sys.exit()
              
	for i, arg in enumerate(sys.argv):
		print(f"Argument {i:>5}: {arg}")	
	
	_durationValue, _color_r, _color_g, _color_b, _color_a = getDurationAndColor(sys.argv[1:])	
	

	# publishing a command on the topic /bolt/cmd_vel which can "talk" to the robot and tell it to move
	pub_action = rospy.Publisher('/bolt/cmd_vel', Twist, queue_size=10)
	
	# publishing a message on the topic /bolt/cmd_duration, and telling the robot how lang to move
	pub_duration = rospy.Publisher('/bolt/cmd_duration', Float32, queue_size=10)	
	
	# publishing a message on the topic /bolt/cmd_color to change the LED-color of the robot	
	pub_color = rospy.Publisher('/bolt/cmd_color', ColorRGBA, queue_size=10)
	
	
	_vel = 0
	_angular_vel = 0
	
	rate = rospy.Rate(10)
	try:
		print(msg)
		while(1):
			key = getKey()
			if key == '' :
				continue
			else:
				if key == 'w' :
					_vel = 20
					_angular_vel = 0

				elif key == 's' :
					_vel = -20
					_angular_vel = 0

				elif key == 'a' :
					_vel = 0
					_angular_vel = -90

				elif key == 'd' :
					_vel = 0
					_angular_vel = 90

				elif key == ' ' or key == 'p' :
					_vel = 0
					_angular_vel = 0

				else:
					if (key == '\x03'):
						break

	
				# controlling the sphero-bolt with this Twist message  
				twist = Twist()
				twist.linear.x = _vel; twist.linear.y = 0.0; twist.linear.z = 0.0
				twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = _angular_vel
				
				boltDuration = Float32()
				boltDuration.data = _durationValue

				boltColor = ColorRGBA()
				boltColor.r = _color_r
				boltColor.g = _color_g
				boltColor.b = _color_b
				boltColor.a = _color_a			

				pub_action.publish(twist)
				pub_duration.publish(boltDuration)	
				pub_color.publish(boltColor)	
							
				rate.sleep()
				rospy.loginfo("sent speed: %d  -- angular-speed: %d", twist.linear.x, twist.angular.z)
				#rospy.loginfo("sent values: (%s)", str(_durationValue)+","+str(_durationValue)+","+str(_color_r)+","+str(_color_g)+","+str(_color_b))	
	except:
		print(e)

	finally:
		twist = Twist()
		twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
		twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
		pub_action.publish(twist)
	
	rospy.loginfo("The program terminated with keyboard interrupt.")
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
