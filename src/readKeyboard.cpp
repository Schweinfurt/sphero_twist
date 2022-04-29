#include <ros/ros.h>

#include <termios.h>
#include <stdio.h>
#include <sys/poll.h>
#include <geometry_msgs/Twist.h>

int spa = 0;
struct termios cooked, raw;

static void keyboardLoop()
{
    char c;
    bool status = false;

    // get the console in raw mode
    tcgetattr(spa, &cooked);
    
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(spa, TCSANOW, &raw);
    
    puts("Reading from keyboard");
    puts("Press Ctrl + C to exit");

    struct pollfd ufd;
    ufd.fd = spa;
    ufd.events = POLLIN;
    
    while (ros::ok())
    {
        // wait for next event on a file descriptor from the keyboard
        int num;

        if ((num = poll(&ufd, 1, 250)) < 0)
        {
            perror("poll():");
            return;
        }
        else if(num > 0)
        {
            if(read(spa, &c, 1) < 0)
            {
                perror("read():");
                return;
            }
        }
        else
        {
            if (status == true)
            {
                status = false;
            }

            continue;
        }

        switch(c)
        {
            case 'q':
                puts("q");
				break;
			case 'w':
				puts("w");
				break;
			case 'e':
				puts("e");
				break;
            default:
                puts("Default");
        }
    }
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"twist_bolt_teleop");
    keyboardLoop();
    ros::spin();
    tcsetattr(spa, TCSANOW, &cooked);
  
    return(0);
}

