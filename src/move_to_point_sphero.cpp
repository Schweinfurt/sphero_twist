//Read the target point on the specific topic and modify the value DURATION 
//Calculate the values: omega, theta and v
//Turn the omega-angle in the correct direction and move forward with speed v 

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <math.h>

class PointController {
	
	private:

		double x_position = 0;
		double y_position = 0;
		double z_rotation = 0;
		bool targetReached = true;
		double goal_x = 0;
		double goal_y = 0;
		bool newPose = true;
		double acceptableError = 5.5;

		ros::NodeHandle n;
		
		//Create a publisher and name the topic
		ros::Publisher robot_vel_pub = n.advertise<geometry_msgs::Twist>("/bolt/cmd_vel", 10);
		
		//Create subscriber
		ros::Subscriber pose_sub = n.subscribe("/bolt/bolt_pose", 10, &PointController::poseCallback, this);
		ros::Subscriber goal_sub = n.subscribe("/bolt/bolt_goal", 10, &PointController::goalCallback, this);
		
		//Tell ROS how fast to run this Node
		ros::Rate loop_rate = 1.25; //equals the rate of the pose publisher
		geometry_msgs::Twist vel_msg;
		
		geometry_msgs::Twist calcVector() {		
			geometry_msgs::Twist vector;
			
			//Apply simple point control algorithm
			double Kv = 0.9; //0.9
			double Kh = 0.2; //0.5
			double v_ij = Kv * sqrt(pow((goal_y - y_position), 2) + pow((goal_x - x_position), 2));
			
			//Limit linear speed
			if(v_ij > 25) {
				v_ij = 25;
			}

			double theta_ij_deg = atan2((goal_x - x_position), (goal_y - y_position)) * 180/M_PI; 
			double theta_i_deg = z_rotation;
			double omega_ij = Kh * (theta_ij_deg - theta_i_deg);
			
			ROS_INFO("----------------------------------------");
			ROS_INFO("Angles: theta_ij:%0.6f, theta_i:%0.6f, omega_ij:%0.6f", theta_ij_deg, theta_i_deg, omega_ij);
			ROS_INFO("Linear Speed v_ij: %0.6f", v_ij);
			ROS_INFO("------------------------------------");
			vector.linear.x = v_ij;
			vector.angular.z = omega_ij;
			return vector;
		}
	
	public:
	
		// Callback function for subscriber. 
		//Data type of /bolt/bolt_pose is geometry_msgs/Pose; consists of position float64 x, float64 y, float64 z and orientation float64 x, float64 y, float64 z, float64 w
		void poseCallback(const geometry_msgs::Pose& msg) {        
			x_position = msg.position.x;
			y_position = msg.position.y;
			z_rotation = msg.orientation.z;
			z_rotation = z_rotation * -1;
			if((x_position > goal_x-acceptableError) && (x_position < goal_x+acceptableError) && (y_position > goal_y-acceptableError) && (y_position < goal_y+acceptableError)) {
				targetReached = true;
				ROS_INFO("Target Reached!");
			} else {
				ROS_INFO("Robot pose: x:%0.6f, y:%0.6f, zrot:%0.6f", x_position, y_position, z_rotation); 
				ROS_INFO("Target: x:%0.6f, y:%0.6f", goal_x, goal_y);
				targetReached = false;
			}
			newPose = true;
		}
		
		
		// Callback function for new target position. 
		void goalCallback(const geometry_msgs::Point& goal_msg) {
			goal_x = goal_msg.x;
			goal_y = goal_msg.y;
			ROS_INFO("new target: x:%0.6f, y:%0.6f", goal_x, goal_y);
		} 
		

		void moveToPoint() {
			while (ros::ok()) {
				//Only compute new linear and angular velocity when new pose was received
				if(!targetReached && (newPose == true)) {
					vel_msg = calcVector();
					newPose = false;
				} else {
					ROS_INFO("newPose: %d", newPose);
					ROS_INFO("Target reached or no new pose");
					vel_msg.linear.x = 0;
					vel_msg.angular.z = 0;
				}
				//Publish linear and angular speed
				robot_vel_pub.publish(vel_msg);
				
				ros::spinOnce();
				loop_rate.sleep();
			}
		}
};


int main(int argc, char **argv) {

	ros::init(argc, argv, "move_to_point_sphero");

	PointController controller;
	controller.moveToPoint();

}


