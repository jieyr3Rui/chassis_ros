#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

import socket
import sys


def serial_in_callback(msg):
    global s
    print("socket in:  ", msg.data)
    s.send(msg.data.encode('utf-8'))
    return

# other code
def main():
    # init ros node
    rospy.init_node("chassis_socket", anonymous=False)
    rospy.Subscriber("serial_in", String, serial_in_callback)
    publisher = rospy.Publisher("serial_out", String, queue_size=100)

    # init socket
    # USB 模式下，机器人默认 IP 地址为 192.168.42.2, 控制命令端口号为 40923
    host = "192.168.1.159"
    port = 40923
    address = (host, int(port))
    # 与机器人控制命令端口建立 TCP 连接
    global s
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print("Connecting...")
    s.connect(address)
    print("Connected!")

    while not rospy.is_shutdown():
        try:
            # 等待机器人返回执行结果
            buf = s.recv(1024)
            serial_out = buf.decode('utf-8')
            print("serial_out: ", serial_out)
            publisher.publish(serial_out)
        except socket.error as e:
            print("Error receiving :", e)
            sys.exit(1)
        # if not len(buf):
        #     break
    # 关闭端口连接
    s.shutdown(socket.SHUT_WR)
    s.close()

if __name__ == '__main__':
	    try:                                                                                                               
                main()
	    except rospy.ROSInterruptException:
		        pass