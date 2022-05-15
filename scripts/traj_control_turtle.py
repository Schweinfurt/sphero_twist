#!/usr/bin/env python3

"""
Created on Wed Feb 16 09:53:05 2022
@author: schmidtm
"""
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import sqrt, atan2
import rospy

class TurtleControl:
    
    def __init__(self):
        rospy.init_node('simple_turtle_controller', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)
        self.pose = Pose()
        self.rate = rospy.Rate(10)
        rospy.loginfo('init done')
        
    def update_pose(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x,4)
        self.pose.y = round(self.pose.y,4)
    
    def euclidean_distance(self, goal_pose):
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
        rospy.loginfo("moving to pose %d %d", goal_x, goal_y)
        goal_pose = Pose()
        goal_pose.x = goal_x
        goal_pose.y = goal_y
        
        vel_msg = Twist();
        
        
        while self.euclidean_distance(goal_pose) >= tolerance:
            rospy.loginfo("Calculating velocities")
            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_pose)
            
            self.velocity_publisher.publish(vel_msg)
            rospy.loginfo("setting linear to %d and angular to %d", vel_msg.linear.x, vel_msg.angular.z)
            self.rate.sleep()
        
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
    
