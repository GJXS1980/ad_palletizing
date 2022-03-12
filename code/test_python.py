#!/usr/bin/python
# -*- coding:utf8 -*-

import rospy
import random
import numpy as np
import time
from std_msgs.msg import Int32, Float32, Float32MultiArray
from ar_track_alvar_msgs.msg import AlvarMarkers


class test:
    def __init__(self):
        rospy.init_node("an_node")
        print(1111111111111)
        self.ar_pose_array = np.zeros((9, 7))
        self.an_current_x = 0.0
        self.an_current_y = 0.0
        self.an_current_z = 0.0
        self.an_grasp_state = 0

        # 浮动计算次数
        self.dete_time = 0
        # 最低高度
        self.min_high = 999
        # 最高高度
        self.max_high = 0

        rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.get_ar_pose)
        # 主控视觉识别话题
        rospy.Subscriber('target_pose', Int32, self.get_target_pose1)

        self.an_control = rospy.Publisher('/Pall_POS_SET', Float32MultiArray, queue_size=5)
        self.an_control_pos = rospy.Subscriber("/Pall_CURR_POS", Float32MultiArray, self.get_an_pose)
        self.an_grasp_control = rospy.Publisher('/Pall_Grasp_Topic', Int32, queue_size=5)
        self.an_grasp_state = rospy.Subscriber("/Pall_GRAB_STATUS", Int32, self.get_an_control)
    
    def get_target_pose1(self, data):
        if data.data == 1 and self.ar_pose_array[1][0] != 0:
            while self.dete_time < 10:
                self.min_high = min(self.min_high, self.ar_pose_array[1][2])
                self.max_high = max(self.max_high, self.ar_pose_array[1][2])	 	         
                self.dete_time += 1
                time.sleep(0.5)
        if self.dete_time >= 10:
            target_pose = Float32MultiArray()
            print("======", self.min_high, self.max_high)
            target_high = (self.min_high + self.max_high) / 2
            target_pose.data = [self.an_current_x, self.an_current_y, self.an_current_z + target_high - 50]
            print(target_pose)
            for i in range(4):
                self.an_control.publish(target_pose)
            #time.sleep(8.0)
            self.dete_time = 0
            #self.an_grasp_control.publish(1)

    def get_target_pose(self, data):
	#print(self.ar_pose_array[1][0])
        if data.data == 1 and self.ar_pose_array[1][0] != 0:
            target_pose = Float32MultiArray()
            #target_pose.data =[self.an_current_x, self.an_current_y, self.an_current_z+self.ar_pose_array[1][2]-80]
            #print(target_pose)
            #for i in range(4):
            #    self.an_control.publish(target_pose)
            #time.sleep(5.0)
            target_pose.data =[self.an_current_x+15+self.ar_pose_array[1][1], self.an_current_y, self.an_current_z]
            print(target_pose)
            for i in range(4):
                self.an_control.publish(target_pose)
            time.sleep(5.0)
            target_pose.data =[self.an_current_x, self.an_current_y+self.ar_pose_array[1][0], self.an_current_z]
            print(target_pose)
            for i in range(4):
                self.an_control.publish(target_pose)
            time.sleep(5.0)
            #target_pose.data =[self.an_current_x, self.an_current_y, self.an_current_z+10]
            #print(target_pose)
            #for i in range(4):
            #    self.an_control.publish(target_pose)
    def get_ar_pose(self, msg):
        #self.ar_pose_array.fill(0)
        for i in range(len(msg.markers)):
            self.ar_pose_array[msg.markers[i].id][0] = round(msg.markers[i].pose.pose.position.x * 1000, 2)
            self.ar_pose_array[msg.markers[i].id][1] = round(msg.markers[i].pose.pose.position.y * 1000, 2)
            self.ar_pose_array[msg.markers[i].id][2] = round(msg.markers[i].pose.pose.position.z * 1000, 2)

            self.ar_pose_array[msg.markers[i].id][3] = msg.markers[i].pose.pose.orientation.w
            self.ar_pose_array[msg.markers[i].id][4] = msg.markers[i].pose.pose.orientation.x
            self.ar_pose_array[msg.markers[i].id][5] = msg.markers[i].pose.pose.orientation.y
            self.ar_pose_array[msg.markers[i].id][6] = msg.markers[i].pose.pose.orientation.z
	#print(self.ar_pose_array)
    def get_an_control(self, data):
        self.an_grasp_state = data.data

    def get_an_pose(self, data):
        self.an_current_x = data.data[0]
        self.an_current_y = data.data[1]
        self.an_current_z = data.data[2]

# if __name__ == '__main__':
#     test = test()
#     rospy.spin()
