// Declaration file 

#pragma once //designed to include the current source file only once in a single compilation.
#ifndef YOUR_PKG_NAME //usd for conditional compiling.
#define YOUR_PKG_NAME
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
#endif  