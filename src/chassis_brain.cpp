#include <ros/ros.h>
#include <chassis_ros/velocity.h>
#include <iostream>
#include <sstream>
#include <string>
#include <std_msgs/String.h>

#define STOP    0
#define START 1
#define STARTING 2

uint8_t get_vel_mode = STOP;

uint8_t chassis_mode = STOP;

std::string in_command        = "command;";
std::string in_mode           = "robot mode free;";
std::string in_get_speed      = "chassis speed ?;";
std::string in_quit           = "quit;";

std::string out_ok            = "ok";
std::string out_command_error = "command exec error";
std::string out_parse_error   = "command format error: command parse error";

std::string get_vel_start        = "vstart";
std::string get_vel_stop         = "vstop";


ros::Publisher  serial_in_pub;
ros::Publisher  current_vel_pub;
ros::Publisher  feedback_pub;


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


// serial_out callback function
void serial_out_callback(const std_msgs::String::ConstPtr& serial_out){
    // std::cout << "serial_out: " << serial_out->data.c_str() << std::endl;
    std::string sc(serial_out->data.c_str());
    // serial data process

    // feedback
    if(sc == out_ok || sc == out_command_error || sc == out_parse_error){
        std::cout << "get feedback: " << sc << std::endl;
        std_msgs::String feedback;
        feedback.data = sc.c_str();
        feedback_pub.publish(feedback);
    }
    // chassis current vel
    else{
        float res[128];
        memset(res, 0, 128*sizeof(float));
        // success
        if(string2array(sc, res)){
            std::cout << "get current vel: " << res[0] << " " << res[1] << " " << res[2] << std::endl;
            chassis_ros::velocity curr_vel;
            curr_vel.x = res[0];
            curr_vel.y = res[1];
            curr_vel.yaw = res[2];
            current_vel_pub.publish(curr_vel);
        }
        // failure
        else{
            std::string sce = "error velocity decode: " + sc;
            std_msgs::String feedback;
            feedback.data = sce.c_str();
            feedback_pub.publish(feedback);
        }
    }
}

// 
void control_vel_callback(chassis_ros::velocity control_vel){
    std::cout << "get control vel x=" << control_vel.x << " y=" << control_vel.y << " z=" << control_vel.yaw << std::endl;
    // serial write
    std::string s0 = "chassis speed x ";
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
    sprintf(cstr,"%.4f",-control_vel.yaw);
    std::string dz(cstr);

    std::string s_out = s0 + dy + s1 + dx + s2 + dz + s3; // !!!
    
    //std::cout << s_out << std::endl;
    std_msgs::String serial_in;
    serial_in.data = s_out.c_str();
    serial_in_pub.publish(serial_in);
}


// command from keyboard to serial
void command_callback(const std_msgs::String::ConstPtr& command){
    std::cout << "command_callback: " << command->data.c_str() << std::endl;
    std::string sc(command->data.c_str());
    // serial data process
    if(sc == get_vel_start){
        std::cout << "get vel start" << std::endl;
        get_vel_mode = START;
    }
    else if(sc == get_vel_stop){
        std::cout << "get vel stop" << std::endl;
        get_vel_mode = STOP;
    }
    else{
        std::cout << "send command" << std::endl;
        std_msgs::String serial_in;
        serial_in.data = command->data.c_str();
        serial_in_pub.publish(serial_in);
    }
}



// chassis serial node
int main(int argc, char** argv){
    // ros init
    ros::init(argc, argv, "chassis_brain");
    ros::NodeHandle n;
    
    ros::Subscriber serial_out_sub = n.subscribe<std_msgs::String>("serial_out", 100, serial_out_callback);
    serial_in_pub   = n.advertise<std_msgs::String>("serial_in", 100);

    ros::Subscriber control_vel_sub = n.subscribe<chassis_ros::velocity>("control_vel", 100, control_vel_callback);
    current_vel_pub = n.advertise<chassis_ros::velocity>("current_vel", 100);

    ros::Subscriber command_sub = n.subscribe<std_msgs::String>("command", 100, command_callback);
    feedback_pub = n.advertise<std_msgs::String>("feedback", 100);


    ros::Rate loop_rate(10);
    while(ros::ok()){
        if(get_vel_mode == START){
            std_msgs::String serial_in;
            serial_in.data = in_get_speed.c_str();
            serial_in_pub.publish(serial_in);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}
