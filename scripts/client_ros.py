#!/usr/bin/env python
#coding:utf-8

import socket
import cv2
import numpy
import time
import sys
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

    def __init__(self):
        rospy.init_node('image_converter', anonymous=True)
        self.host_ip = rospy.get_param("~host_ip", "192.168.3.8")
        self.host_port = rospy.get_param("~host_port", "9999")
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("usb_cam/image_raw", Image, self.callback)
        self.address = (self.host_ip, self.host_port)
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect(self.address)
        except socket.error as msg:
            print(msg)
            sys.exit(1)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            frame = cv2.flip(cv_image, 1)
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]
            result, imgencode = cv2.imencode('.jpg', frame, encode_param)
            data = numpy.array(imgencode)
            stringData = data.tostring()
            # 先发送要发送的数据的长度
            # ljust() 方法返回一个原字符串左对齐,并使用空格填充至指定长度的新字符串
            self.sock.send(str.encode(str(len(stringData)).ljust(16)))

            self.sock.send(stringData)
            # 读取服务器返回值
            receive = self.sock.recv(1024)
            if len(receive):
                print("send img time: ", str(receive) + 's')
        except:
            self.sock.close()
            sys.exit(1)


def main(args):
    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
    ic.sock.close()

if __name__ == '__main__':
    main(sys.argv)
