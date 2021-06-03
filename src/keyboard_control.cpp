#include <ros/ros.h>
#include <chassis_ros/velocity.h>
#include <std_msgs/String.h>
#include <iostream>
#include <sstream>
#include <string>




void feedback_callback(const std_msgs::String::ConstPtr& feedback){
    std::cout << "i heard feedback: " << feedback->data.c_str() << std::endl;
}


// a simple keyboard control
int main(int argc, char** argv)
{
    // ros init
    ros::init(argc, argv, "keyboard_control");
    ros::NodeHandle n;
    ros::Publisher control_vel_pub = n.advertise<chassis_ros::velocity>("chassis_control_vel", 100);
    ros::Publisher command_pub = n.advertise<std_msgs::String>("command", 10);
    ros::Subscriber feedback_sub = n.subscribe<std_msgs::String>("feedback", 10, feedback_callback);

    chassis_ros::velocity control_vel;
    std_msgs::String command;

    control_vel.yaw = 0;
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        std::cout << "input: " << std::endl;

        std::string skey;
        getline(std::cin, skey);
        //input one char chassis velocity control 
        if(skey.size() == 1){ 
            switch (skey[0])
            {
            case 'd': // forward
                control_vel.x = 0;
                control_vel.y = 0.2;
                break;
            
            case 's': // left
                control_vel.x = -0.2;
                control_vel.y = 0;
                break;

            case 'a': // back
                control_vel.x = 0;
                control_vel.y = -0.2;
                break;

            case 'w': // right
                control_vel.x = 0.2;
                control_vel.y = 0;
                break;

            case 'q': // stop
                control_vel.x = 0.2;
                control_vel.y = 0;
                break;

            default: // stop
                control_vel.x = 0;
                control_vel.y = 0;
                break;
            }
            std::cout << "chassis velocity control " << skey[0] << std::endl;
            control_vel_pub.publish(control_vel);
            ros::spinOnce();
        }

        // input a command
        else if(skey.size() > 1){
            command.data = skey.c_str();
            std::cout << "command " << command.data << std::endl;
            command_pub.publish(command);
            ros::spinOnce();
        }



        loop_rate.sleep();
    }

    return 0;
}
