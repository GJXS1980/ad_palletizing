#!/usr/bin/env python
# -*- coding:utf8 -*-

import rospy
import sys
import math
import time
import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Twist
from math import copysign
import json
import numpy as np
import time
import cv2
import paho.mqtt.client as mqtt
from threading import *
import paho.mqtt.client as mqtt
from collections import OrderedDict
from std_msgs.msg import Int32, Float32, Float32MultiArray

class Vision_Mqtt():
    def __init__(self):
        self.mqtt_host = "192.168.3.6"
        self.mqtt_port = 50001
        self.mqttClient = mqtt.Client()
        self.AN_Vision_Type = "Unknow"
        self.AN_Vision_ID = 0
        #self.start_mqtt()

    # 连接MQTT服务器
    def on_mqtt_connect(self):
        self.mqttClient.connect(self.mqtt_host, self.mqtt_port, 60)

    # subscribe 消息
    def on_subscribe(self):
        self.mqttClient.subscribe("/HG_DEV/Vision_MSG", 2)
        self.mqttClient.on_message = self.on_message_come  # 消息到来处理函数

    def start_mqtt(self):
        self.on_mqtt_connect()
        self.on_subscribe()
        self.mqttClient.loop_start()

    # publish 消息
    def on_publish(self, payload, topic="/HG_DEV/Vision_REQ", qos=2):
        print("pub msg: %s to %s" % (payload, topic))
        self.mqttClient.publish(topic, payload, qos)

    # 消息处理函数
    def on_message_come(self, lient, userdata, msg):
        print("from: " + msg.topic + " " + ":" + msg.payload)
        sub_msg = json.loads(msg.payload.encode('utf-8'))
        if sub_msg["name"] == "ZK" and sub_msg["dir"] == "AD":
            self.AN_Vision_Type = sub_msg["action"]
            self.AN_Vision_ID = sub_msg["target_id"]

class AN_Vision():
    def __init__(self):
        rospy.init_node("an_visvion_node")
        self.host_ip = rospy.get_param("~host_ip", "192.168.3.8")
        self.host_port = rospy.get_param("~host_port", "9999")
        # 视觉识别模式,0:AGV上没有码,盲放; 1:AGV上有码,识别放置
        self.vision_mode = rospy.get_param("~vision_mode", "0")
        # 视觉误差补偿
        self.x_dis = rospy.get_param("~x_dis", 15) # x方向,mm
        self.y_dis = rospy.get_param("~y_dis", 5) # y方向,mm
        self.z_dis = rospy.get_param("~z_dis", -35) # z方向,mm
        self.an_current_x = 0.0
        self.an_current_y = 0.0
        self.an_current_z = 0.0
        
        self.min_high = 9999
        self.max_high = 0
        self.dete_time = 0

        self.an_vision_req_msg = OrderedDict()
        self.an_vision_req_msg["name"] = "AD"
        self.an_vision_req_msg["dir"] = "ZK"
        self.an_vision_req_msg["action"] = "SL"
        self.an_vision_req_msg["target_pose"] = [0.0, 0.0, 0.0]
        self.an_vision_req_json = json.dumps(self.an_vision_req_msg)

        self.ar_pose_array = np.zeros((9, 7))
        self.push_state = 0 

        #   mqtt连接
        self.AN_Vision_Mqtt = Vision_Mqtt()
        self.AN_Vision_Mqtt.mqtt_host = self.host_ip
        self.AN_Vision_Mqtt.mqtt_port = self.host_port
        self.AN_Vision_Mqtt.start_mqtt()

        rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.get_ar_pose)  #   获取AR码位姿

        rospy.Subscriber("get_push_pose", Int32, self.get_push_state)   #   获取放置状态

        self.an_target_pose = rospy.Publisher('/target_pose', Float32MultiArray, queue_size=5)  # 发送目标位姿
        self.push_target_pose = rospy.Publisher('/push_target_pose', Float32MultiArray, queue_size=5)   #   发送放置目标位姿
        self.an_control_pos = rospy.Subscriber("/Pall_CURR_POS", Float32MultiArray, self.get_an_pose)   #   获取当前岸吊位姿

    # 岸吊放置状态
    def get_push_state(self, data):
        if data.data == 1:
            self.push_state = 1

    # 岸吊当前位姿
    def get_an_pose(self, data):
        self.an_current_x = data.data[0]
        self.an_current_y = data.data[1]
        self.an_current_z = data.data[2]

    # AR码位姿,返回位姿（xyz,wxyz）的四舍五入值
    def get_ar_pose(self, msg):
        self.ar_pose_array.fill(0)
        for i in range(len(msg.markers)):
            self.ar_pose_array[msg.markers[i].id][0] = round(msg.markers[i].pose.pose.position.x*1000, 2)
            self.ar_pose_array[msg.markers[i].id][1] = round(msg.markers[i].pose.pose.position.y*1000, 2)
            self.ar_pose_array[msg.markers[i].id][2] = round(msg.markers[i].pose.pose.position.z*1000, 2)

            self.ar_pose_array[msg.markers[i].id][3] = msg.markers[i].pose.pose.orientation.w
            self.ar_pose_array[msg.markers[i].id][4] = msg.markers[i].pose.pose.orientation.x
            self.ar_pose_array[msg.markers[i].id][5] = msg.markers[i].pose.pose.orientation.y
            self.ar_pose_array[msg.markers[i].id][6] = msg.markers[i].pose.pose.orientation.z

        if self.AN_Vision_Mqtt.AN_Vision_ID == 1 and self.ar_pose_array[0][2] != 0.0:
            #   识别船运动的最大和最小的幅度（z方向）
            if self.dete_time < 10:
                self.min_high = min(self.ar_pose_array[0][2], self.min_high)
                self.max_high = max(self.ar_pose_array[0][2], self.max_high)
                self.dete_time += 1
                time.sleep(0.2)

            if self.dete_time >= 10:
                #    取最大和最小幅度平均值加上在z方向的补偿
                target_high = (self.min_high + self.max_high) / 2 + self.z_dis
                #   求解公式：target_x = current_x + ar_pose_x + dis_x,etc.
                target_pose =[self.an_current_x+self.ar_pose_array[0][1]+self.x_dis, self.an_current_y+self.ar_pose_array[0][0]+self.y_dis, self.an_current_z+target_high]

                self.an_vision_req_msg["action"] = self.AN_Vision_Mqtt.AN_Vision_Type
                self.an_vision_req_msg["target_pose"] = target_pose#[self.ar_pose_array[0][0], self.ar_pose_array[0][1], self.ar_pose_array[0][2]]
                self.an_vision_req_json = json.dumps(self.an_vision_req_msg)
                self.AN_Vision_Mqtt.on_publish(self.an_vision_req_json, "/HG_DEV/Vision_REQ", 2)
                # data = [self.ar_pose_array[0][0], self.ar_pose_array[0][1], self.ar_pose_array[0][2]]
                target_pose_data = Float32MultiArray(data=target_pose)
                #   发送目标位姿（连续发布4次）
                for i in range(4):
                    self.an_target_pose.publish(target_pose_data)

        if self.AN_Vision_Mqtt.AN_Vision_ID == 2 and self.ar_pose_array[1][2] != 0.0:
            if self.dete_time < 10:
                    self.min_high = min(self.ar_pose_array[1][2], self.min_high)
                    self.max_high = max(self.ar_pose_array[1][2], self.max_high)
                    self.dete_time += 1
                    time.sleep(0.2)
            
            if self.dete_time >= 10:
                target_high = (self.min_high + self.max_high) / 2 + self.z_dis

                target_pose =[self.an_current_x+self.ar_pose_array[1][1]+self.x_dis, self.an_current_y+self.ar_pose_array[1][0]+self.y_dis, self.an_current_z+target_high]
                   
                self.AN_Vision_Mqtt.AN_Vision_ID = 0
                self.dete_time = 0
                self.min_high = 9999
                self.max_high = 0
                
                self.an_vision_req_msg["action"] = self.AN_Vision_Mqtt.AN_Vision_Type
                self.an_vision_req_msg["target_pose"] = target_pose#[self.ar_pose_array[0][0], self.ar_pose_array[0][1], self.ar_pose_array[0][2]]
                self.an_vision_req_json = json.dumps(self.an_vision_req_msg)
                self.AN_Vision_Mqtt.on_publish(self.an_vision_req_json, "/HG_DEV/Vision_REQ", 2)
                # data = [self.ar_pose_array[0][0], self.ar_pose_array[0][1], self.ar_pose_array[0][2]]
                target_pose_data = Float32MultiArray(data=target_pose)
                for i in range(4):
                    self.an_target_pose.publish(target_pose_data)

        if self.AN_Vision_Mqtt.AN_Vision_ID == 3 and self.ar_pose_array[2][2] != 0.0:
        
            if self.dete_time < 10:
                self.min_high = min(self.ar_pose_array[2][2], self.min_high)
                self.max_high = max(self.ar_pose_array[2][2], self.max_high)
                self.dete_time += 1
                time.sleep(0.2)
                 
            if self.dete_time >= 10:
                target_high = (self.min_high + self.max_high) / 2 + self.z_dis
                
                target_pose =[self.an_current_x+self.ar_pose_array[2][1]+self.x_dis, self.an_current_y+self.ar_pose_array[2][0]+self.y_dis, self.an_current_z+target_high]

                self.AN_Vision_Mqtt.AN_Vision_ID = 0
                self.dete_time = 0
                self.min_high = 9999
                self.max_high = 0
                
                self.an_vision_req_msg["action"] = self.AN_Vision_Mqtt.AN_Vision_Type
                self.an_vision_req_msg["target_pose"] = target_pose#[self.ar_pose_array[0][0], self.ar_pose_array[0][1], self.ar_pose_array[0][2]]
                self.an_vision_req_json = json.dumps(self.an_vision_req_msg)
                self.AN_Vision_Mqtt.on_publish(self.an_vision_req_json, "/HG_DEV/Vision_REQ", 2)
                # data = [self.ar_pose_array[0][0], self.ar_pose_array[0][1], self.ar_pose_array[0][2]]
                target_pose_data = Float32MultiArray(data=target_pose)
                for i in range(4):
                    self.an_target_pose.publish(target_pose_data)
        
        if self.vision_mode == 1 and self.ar_pose_array[6][2] != 0.0 and self.push_state == 1:
            target_pose =[self.an_current_x+self.ar_pose_array[6][1]+self.x_dis, self.an_current_y+self.ar_pose_array[6][0]+self.y_dis, self.an_current_z+self.ar_pose_array[6][2]+self.z_dis]

            self.an_vision_req_msg["action"] = self.AN_Vision_Mqtt.AN_Vision_Type
            self.an_vision_req_msg["target_pose"] = target_pose#[self.ar_pose_array[0][0], self.ar_pose_array[0][1], self.ar_pose_array[0][2]]
            self.an_vision_req_json = json.dumps(self.an_vision_req_msg)
            self.AN_Vision_Mqtt.on_publish(self.an_vision_req_json, "/HG_DEV/Vision_REQ", 2)
            # data = [self.ar_pose_array[0][0], self.ar_pose_array[0][1], self.ar_pose_array[0][2]]
            target_pose_data = Float32MultiArray(data=target_pose)
            for i in range(4):
                self.push_target_pose.publish(target_pose_data)
            
            self.push_state = 0

if __name__ == '__main__':
    an_start = AN_Vision()
    rospy.spin()
