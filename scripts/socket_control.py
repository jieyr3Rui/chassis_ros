#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from chassis_ros.msg import velocity

import socket
import sys


# other code
def main():
    # init ros node
    rospy.init_node("socket_control", anonymous=False)
    command_pub     = rospy.Publisher("command", String, queue_size=10)
    control_vel_pub = rospy.Publisher("control_vel", velocity, queue_size=10)
    policy_mode_pub = rospy.Publisher("policy_mode", String, queue_size=10)

    # init socket
    host='192.168.1.126'
    port=12344
    addr=(host,port)
    tcpCliSock=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    buffsize=1024  
    print("connecting...")
    tcpCliSock.connect(addr)
    print("connected!")

    while not rospy.is_shutdown():
        try:
            scm = tcpCliSock.recv(buffsize).decode('utf-8')
            if len(scm) <= 1:
                print("error scm")
                continue

            print(len(scm) , " ", scm)

            # control_vel
            if (scm[0] == "v") & (len(scm) == 2):
                print("control_vel")
                # velocity setting
                vel = velocity()
                vel.x = 0
                vel.y = 0
                vel.yaw = 0
                if scm[1] == "w":
                    vel.y = 0.2
                elif scm[1] == "s":
                    vel.y = -0.2
                elif scm[1] == "a":
                    vel.x = -0.2
                elif scm[1] == "d":
                    vel.x = 0.2
                elif scm[1] == "q":
                    vel.yaw = 30
                elif scm[1] == "e":
                    vel.yaw = -30     
                control_vel_pub.publish(vel)
                continue

            # policy
            elif scm[0] == "p":
                print("policy_mode")
                policy_mode_pub.publish(scm[1:len(scm)])
                continue
            
            elif scm[0] == "c":
                print("command;")
                command_pub.publish(scm[1:len(scm)])
                continue

            else:
                print("unknow control")
                continue

        except socket.error as e:
            print("Error receiving :", e)
            sys.exit(1)

    # close socket
    tcpCliSock.shutdown(socket.SHUT_WR)
    tcpCliSock.close()

if __name__ == '__main__':
    try:                                                                                                               
        main()
    except rospy.ROSInterruptException:
        pass