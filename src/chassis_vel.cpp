#include <ros/ros.h>
#include <serial/serial.h>
#include <chassis_ros/velocity.h>
#include <iostream>
#include <sstream>
#include <string>
#include <stdlib.h>
#include <std_msgs/String.h>

serial::Serial sp;
// function string to float array
int string2array(std::string s, float res[]){
    // std::cout << "s = " << s << std::endl;
	int i = 0, j = 0, k = 0;
	for(j = 0; j < s.size(); j++)
    {   // not a number
		if(s[j] == ' ')
        {
			std::string temp = s.substr(i, j-i);
			// std::cout << temp << std::endl;
			res[k++] = atof(temp.c_str());
			i = j+1;
            // std::cout << k << " th = " << res[k-1] << std::endl;
		}
	}
    if(k != 7) // not a 7 floats string
    {   
        memset(res, 0, 7*sizeof(float));
        // std::cout << "here0" << std::endl;
        return 0;
    }
    // std::cout << "here1" << std::endl;
    return 1;
}

void control_vel_callback(chassis_ros::velocity control_vel){
    std::cout << "get control vel x=" << control_vel.x << " y=" << control_vel.y << std::endl;
    // serial write
    std::string s0 = "chassis speed x ";\
    std::string s1 = " y ";
    std::string s2 = " z ";
    std::string s3 = ";";
    
    char cstr[20];
    memset(cstr, 0, 20*sizeof(char));
    sprintf(cstr,"%.4f",control_vel.x);
    std::string dx(cstr);

    memset(cstr, 0, 20*sizeof(char));
    sprintf(cstr,"%.4f",control_vel.y);
    std::string dy(cstr);

    memset(cstr, 0, 20*sizeof(char));
    sprintf(cstr,"%.4f",control_vel.yaw);
    std::string dz(cstr);

    std::string s_out = s0 + dx + s1 + dy + s2 + dz + s3;
    
    std::cout << s_out << std::endl;
    sp.write((uint8_t*)s_out.c_str(), s_out.size());
}

void command_callback(const std_msgs::String::ConstPtr& command){
    std::cout << "i heard command: " << command->data.c_str() << std::endl;
    std::string sc(command->data.c_str());
    sp.write((uint8_t*)sc.c_str(), sc.size());
}

int main(int argc, char** argv)
{
    // ros init
    ros::init(argc, argv, "chassis_vel");
    ros::NodeHandle n;
    ros::Publisher vel_pub  = n.advertise<chassis_ros::velocity>("chassis_current_vel", 100);
    ros::Subscriber vel_sub = n.subscribe<chassis_ros::velocity>("chassis_control_vel", 100, control_vel_callback);
    ros::Subscriber command_sub = n.subscribe<std_msgs::String>("command", 10, command_callback);
    ros::Publisher feedback_pub = n.advertise<std_msgs::String>("feedback", 10);

    // serial port init

    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    sp.setPort("/dev/ttyUSB0");
    sp.setBaudrate(115200);
    sp.setTimeout(to);
    try
    {
        sp.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }
    
    if(sp.isOpen())
    {
        ROS_INFO_STREAM("/dev/ch340 is opened.");
    }
    else
    {
        return -1;
    }
    


    std::string in_command = "command;";
    std::string in_mode = "robot mode free;";
    std::string in_get_speed = "chassis speed ?;";
    
    std::string out_ok = "ok";
    std::string out_command_error = "command exec error";
    std::string out_parse_error = "command format error: command parse error";


    // // define a buffer
    // uint8_t buffer[128];
    // memset(buffer, 0, 128*sizeof(uint8_t));

    // // send command setting
    // sp.write((uint8_t*)in_command.c_str(), in_command.size());
    // while(sp.available() == 0){} // wait for feedback
    // sp.read(buffer, sp.available());


    // // send mode setting
    // sp.write((uint8_t*)in_mode.c_str(), in_mode.size());
    // while(sp.available() == 0){}
    // sp.read(buffer, sp.available());
    

    ros::Rate loop_rate(1);
    while(ros::ok())
    {
        sp.write((uint8_t*)in_get_speed.c_str(), in_get_speed.size());
        size_t n = sp.available();
        if(n!=0)
        {
            uint8_t buffer[128];
            memset(buffer, 0, 128*sizeof(uint8_t));
            // read buffer and print
            n = sp.read(buffer, n);
            
        
            std::string s((char*)buffer);

            // try to convert string to float32 array
            float chassis_vel[7];
            memset(chassis_vel, 0, 7*sizeof(float));
            int trans_res = string2array(s, chassis_vel);
            
            // chassis velocity 
            if(trans_res == 1){
                std::cout << "serial get chassis_vel: " << std::endl;
                for(int i=0; i<7; i++)
                {
                    std::cout << chassis_vel[i] << " ";
                }
                std::cout << std::endl;

                chassis_ros::velocity curr_vel;
                curr_vel.x = chassis_vel[0];
                curr_vel.y = chassis_vel[1];
                curr_vel.yaw = chassis_vel[2];

                vel_pub.publish(curr_vel);
            }
            else{
                // std::cout << "serial get feedback: " << s.c_str() << std::endl;
                std_msgs::String feedback;
                feedback.data = s.c_str();
                feedback_pub.publish(feedback);
            }

            ros::spinOnce();
        }
        ros::spinOnce();
        loop_rate.sleep();

    }
    

    sp.close();
    return 0;
}

