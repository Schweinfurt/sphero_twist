// this file is only an example! You need to modify it if you use it!  

#include <ros/ros.h> // including the ros header file

/* defining the class */
class YourPKGName
{
    public:
        YourPKGName(ros::NodeHandle &nh, ros::NodeHandle &pnh); //constructor method
        ~YourPKGName(); // distructor method
        void runOnce(); // runOnce method to control the flow of program
    private:
        ros::NodeHandle nh_; // Defining the ros NodeHandle variable for registrating the same with the master
};

