#!/usr/bin/env python3

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import sqrt, atan2
import rospy

class TurtleControl:
    
    def __init__(self):
        # Creates a node with name 'turtlebot_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('simple_turtle_controller', anonymous=True)
        
        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
        # when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)
        
        self.pose = Pose()
        self.rate = rospy.Rate(10)
        rospy.loginfo('init done')
        
    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.pose = data
        self.pose.x = round(self.pose.x,4)
        self.pose.y = round(self.pose.y,4)
    
    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        xsquared = pow(goal_pose.x - self.pose.x,2)
        ysquared = pow(goal_pose.y - self.pose.y,2)
        return sqrt(xsquared + ysquared)
    
    def linear_vel(self, goal_pose, constant = 1.5):
        # simple gain control
        return constant * self.euclidean_distance(goal_pose)
    
    def angular_vel(self, goal_pose, constant = 6):
        return constant * (self.steering_angle(goal_pose) - self.pose.theta )
        
    def steering_angle(self, goal_pose):
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)
    
    def move_to_goal(self, goal_x, goal_y, tolerance=1):
        """Moves the turtle to the goal."""
        rospy.loginfo("moving to pose %d %d", goal_x, goal_y)
        goal_pose = Pose()
        goal_pose.x = goal_x
        goal_pose.y = goal_y
        
        vel_msg = Twist();
        
        
        while self.euclidean_distance(goal_pose) >= tolerance:
            # Porportional controller.
            
            # Linear velocity in the x-axis.
            rospy.loginfo("Calculating velocities")
            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            
            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_pose)
            
            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)
            rospy.loginfo("setting linear to %d and angular to %d", vel_msg.linear.x, vel_msg.angular.z)
            
            # Publish at the desired rate.
            self.rate.sleep()
            
        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)       
    
if __name__ == '__main__':
    try:
        controller = TurtleControl()
        controller.move_to_goal(1,3)
        controller.move_to_goal(5,2)
        controller.move_to_goal(2,10)
    except rospy.ROSInterruptException:
        pass
    
