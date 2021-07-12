#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from chassis_ros.msg import velocity

import socket
import sys

def serial_in_callback(msg):
    global tcpCliSock
    tcpCliSock.send(msg.data.encode('utf-8'))

def serial_out_callback(msg):
    global tcpCliSock
    tcpCliSock.send(msg.data.encode('utf-8'))


def main():
    global tcpCliSock
    # init ros node
    rospy.init_node("socket_debug", anonymous=False)

    rospy.Subscriber("serial_in", String, serial_in_callback)
    rospy.Subscriber("serial_out", String, serial_out_callback)

    # init socket
    host='192.168.1.126'
    port=12355
    addr=(host,port)
    tcpCliSock=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    buffsize=1024  
    print("connecting...")
    tcpCliSock.connect(addr)
    print("connected!")

    rospy.spin()
    # close socket
    tcpCliSock.shutdown(socket.SHUT_WR)
    tcpCliSock.close()

if __name__ == '__main__':
    try:                                                                                                               
        main()
    except rospy.ROSInterruptException:
        pass