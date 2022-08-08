#!/usr/bin/python
# -*- coding:utf8 -*-

import paho.mqtt.client as mqtt
from collections import OrderedDict
import json
import rospy
import random
import time
from threading import *
from std_msgs.msg import Int32, Float32, Float32MultiArray

class AN_Mqtt():
    def __init__(self):
        self.mqtt_host = "192.168.10.124"
        self.mqtt_port = 50002
        self.mqttClient = mqtt.Client()
        self.AN_Mission_Msg = 0
        #self.start_mqtt()

    # 连接MQTT服务器
    def on_mqtt_connect(self):
        self.mqttClient.connect(self.mqtt_host, self.mqtt_port, 60)

    # subscribe 消息
    def on_subscribe(self):
        self.mqttClient.subscribe("/HG_DEV/ZK_ALL", 2)
        self.mqttClient.on_message = self.on_message_come  # 消息到来处理函数

    def start_mqtt(self):
        self.on_mqtt_connect()
        self.on_subscribe()
        self.mqttClient.loop_start()

    # publish 消息
    def on_publish(self, payload, topic="/HG_DEV/ZK_ALL_REQ", qos=2):
        print("pub msg: %s to %s" % (payload, topic))
        self.mqttClient.publish(topic, payload, qos)

    # 消息处理函数
    def on_message_come(self, lient, userdata, msg):
        print("from: " + msg.topic + " " + ":" + msg.payload)
        sub_msg = json.loads(msg.payload.encode('utf-8'))
        if sub_msg["name"] == "ZK" and sub_msg["dir"] == "AD" and sub_msg["mission"] == 1:
            self.AN_Mission_Msg = 1

class AN_Mission_Test:
    def __init__(self):
        rospy.init_node("an_node")
        self.host_ip = rospy.get_param("~host_ip", "192.168.10.124")
        self.host_port = rospy.get_param("~host_port", "50002")
        # 视觉识别模式,0:AGV上没有码,盲放; 1:AGV上有码,识别放置
        self.vision_mode = rospy.get_param("~vision_mode", "0")

        # 抓取集装箱1识别点
        self.grasp_x1 = int(rospy.get_param("~grasp_x1", "80"))
        self.grasp_y1 = int(rospy.get_param("~grasp_y1", "490"))
        self.grasp_z1 = int(rospy.get_param("~grasp_z1", "10"))
        
        # 放置集装箱1识别点
        self.push_x1 = int(rospy.get_param("~push_x1", "200"))
        self.push_y1 = int(rospy.get_param("~push_y1", "60"))
        self.push_z1 = int(rospy.get_param("~push_z1", "30"))

        # 默认集装箱1放置点
        self.real_push_x1 = int(rospy.get_param("~real_push_x1", "200"))
        self.real_push_y1 = int(rospy.get_param("~real_push_y1", "60"))
        self.real_push_z1 = int(rospy.get_param("~real_push_z1", "250"))

        # 抓取集装箱2识别点
        self.grasp_x2 = int(rospy.get_param("~grasp_x2", "80"))
        self.grasp_y2 = int(rospy.get_param("~grasp_y2", "490"))
        self.grasp_z2 = int(rospy.get_param("~grasp_z2", "10"))
        
        # 放置集装箱2识别点
        self.push_x2 = int(rospy.get_param("~push_x2", "200"))
        self.push_y2 = int(rospy.get_param("~push_y2", "60"))
        self.push_z2 = int(rospy.get_param("~push_z2", "30"))

        # 默认集装箱2放置点
        self.real_push_x2 = int(rospy.get_param("~real_push_x2", "200"))
        self.real_push_y2 = int(rospy.get_param("~real_push_y2", "60"))
        self.real_push_z2 = int(rospy.get_param("~real_push_z2", "250"))

        self.an_req_msg = OrderedDict()
        self.an_req_msg["name"] = "AD"
        self.an_req_msg["dir"] = "ZK"
        self.an_req_msg["mission"] = 1
        self.an_req_msg["state"] = "working"
        self.an_req_json = json.dumps(self.an_req_msg)

        self.send_time = 0
        self.wait_time = 0
        self.an_current_x = 0.0
        self.an_current_y = 0.0
        self.an_current_z = 0.0
        self.an_grasp_state = 0
        self.an_target_pose = [0.0, 0.0, 0.0]
        # 放置点
        self.push_target_pose = [0.0, 0.0, 0.0]
        self.an_control = rospy.Publisher('/Pall_POS_SET', Float32MultiArray, queue_size=5)
        self.an_control_pos = rospy.Subscriber("/Pall_CURR_POS", Float32MultiArray, self.get_an_pose)
        self.an_grasp_control = rospy.Publisher('/Pall_Grasp_Topic', Int32, queue_size=5)
        self.an_grasp_state = rospy.Subscriber("/Pall_GRAB_STATUS", Int32, self.get_an_control)

        # 主控视觉识别话题
        rospy.Subscriber('target_pose', Float32MultiArray, self.get_target_pose)
        rospy.Subscriber('push_target_pose', Float32MultiArray, self.get_push_data)
        self.set_push_state = rospy.Publisher('/get_push_pose', Int32, queue_size=5)

        self.AN_Mqtt = AN_Mqtt()
        self.AN_Mqtt.mqtt_host = self.host_ip
        self.AN_Mqtt.mqtt_port = self.host_port
        self.AN_Mqtt.start_mqtt()

        self.th = Thread(target=self.get_mission)
        self.th.start()
    
    def get_target_pose(self, data):
        self.an_target_pose = data.data
        print(data.data)
    
    def get_push_data(self, data):
        self.push_target_pose = data.data
        print(data.data)

    def get_mission(self):
        while True:
            if self.AN_Mqtt.AN_Mission_Msg == 1:
                self.AN_Mqtt.AN_Mission_Msg = 0
                self.an_req_msg["state"] = "working"
                self.an_req_json = json.dumps(self.an_req_msg)
                self.AN_Mqtt.on_publish(self.an_req_json, "/HG_DEV/ZK_ALL_REQ", 2)
                time.sleep(0.5)
                print("开始上货")
                # 如果AGV上使用识别放置,先到放置点识别点识别
                if self.vision_mode == 1:
                    # 发布获取放置识别点信息
                    self.set_push_state.publish(1)
                    # 移动到放置识别点
                    if self.set_an_pose([self.push_x1, self.push_y1, self.push_z1]):
                        # 识别放置坐标并记录(普通ar码识别)
                        try:
                            data = rospy.wait_for_message("push_target_pose", Float32MultiArray)
                            self.push_target_pose = data.data
                        except:
                            pass

                # 移动到抓取识别点
                if self.set_an_pose([self.grasp_x1, self.grasp_y1, self.grasp_z1]):
                    try:
                        # 识别抓取坐标并执行(从主控界面发送视觉识别物体编号,并接受坐标话题)
                        data = rospy.wait_for_message("target_pose", Float32MultiArray)
                        if self.set_an_pose(data.data):
                            #time.sleep(1.5)
                            self.set_an_grasp_control(1)
                        #time.sleep(1.0)
                        #while True:
                            #if self.an_grasp_state:  # 已经抓上来
                            #    break
                            #else:
                                #self.set_an_grasp_control(1)
                                #time.sleep(1.0)
                        # 往上提起物体并;移动到放置识别点
                        if self.set_an_pose([0.0, 0.0, 30.0]):
                            if self.set_an_pose([self.push_x1, self.push_y1, self.push_z1]):
                                if self.vision_mode == 1 and self.push_target_pose[2]!=0.0:
                                    if self.set_an_pose(self.push_target_pose): # 识别放置点
                                        print("push target")
                                else:
                                    if self.set_an_pose([self.real_push_x1, self.real_push_y1, self.real_push_z1]): # 盲放位置点
                                        print("push target")
                                time.sleep(1.0)
                                self.set_an_grasp_control(0)
                                time.sleep(1.0)
                                while True:
                                    if not self.an_grasp_state:  # 已经放开
                                        break
                                    else:
                                        self.set_an_grasp_control(0)

                                if self.set_an_pose([0.0, 0.0, 20.0]):  # 放上去后先往上
                                    if self.set_an_pose([self.grasp_x2, self.grasp_y2, self.grasp_z2]):
                                        try:
                                            # 识别抓取坐标并执行(从主控界面发送视觉识别物体编号,并接受坐标话题)
                                            data1 = rospy.wait_for_message("target_pose1", Float32MultiArray)
                                            if self.set_an_pose(data1.data):
                                                #time.sleep(1.5)
                                                self.set_an_grasp_control(1)
                                            #time.sleep(1.0)
                                            #while True:
                                                #if self.an_grasp_state:  # 已经抓上来
                                                #    break
                                                #else:
                                                    #self.set_an_grasp_control(1)
                                                    #time.sleep(1.0)
                                            # 往上提起物体并;移动到放置识别点
                                            if self.set_an_pose([0.0, 0.0, 30.0]):
                                                if self.set_an_pose([self.push_x2, self.push_y2, self.push_z2]):
                                                    if self.vision_mode == 1 and self.push_target_pose[2]!=0.0:
                                                        if self.set_an_pose(self.push_target_pose): # 识别放置点
                                                            print("push target")
                                                    else:
                                                        if self.set_an_pose([self.real_push_x2, self.real_push_y2, self.real_push_z2]): # 盲放位置点
                                                            print("push target")
                                                    time.sleep(1.0)
                                                    self.set_an_grasp_control(0)
                                                    time.sleep(1.0)
                                                    while True:
                                                        if not self.an_grasp_state:  # 已经放开
                                                            break
                                                        else:
                                                            self.set_an_grasp_control(0)

                                                    if self.set_an_pose([0.0, 0.0, 20.0]):  # 放上去后先往上
                                                        self.an_req_msg["state"] = "finish"
                                                        self.an_req_json = json.dumps(self.an_req_msg)
                                                        self.AN_Mqtt.on_publish(self.an_req_json, "/HG_DEV/ZK_ALL_REQ", 2)
                                        except:
                                            pass
                    except:
                        pass
            
                # if self.set_an_pose([150.0, 650.0, 510.0]):  # 抓取点
                #     self.set_an_grasp_control(1)
                # while True:
                #     if self.an_grasp_state:  # 已经抓上来
                #         break

                # if self.set_an_pose([0.0, 0.0, 300.0]):  # 抓到后先往上
                #     if self.set_an_pose([130.0, 75.0, 520.0]):  # 放置点
                #         self.set_an_grasp_control(0)
                #     while True:
                #         if not self.an_grasp_state:  # 已经放上去
                #             if self.set_an_pose([0.0, 0.0, 100.0]):  # 放上去后先往上
                #                 break

    def get_an_pose(self, data):
        self.an_current_x = data.data[0]
        self.an_current_y = data.data[1]
        self.an_current_z = data.data[2]

    def set_an_pose(self, data):
        if data[0] != 0.0 or data[1] != 0.0:  # x,y不为0,先移动x,y再移动z
            move_xy = [data[0], data[1], self.an_current_z]
            target_pose = Float32MultiArray()
            target_pose.data = move_xy
            print(target_pose)
            for i in range(4):
                self.an_control.publish(target_pose)
            while True:
                if abs(self.an_current_x - data[0]) < 5 and abs(self.an_current_y - data[1]) < 5:
                    break
            move_z = [data[0], data[1], data[2]]
            target_pose = Float32MultiArray(data=move_z)
            print(target_pose)
            for i in range(4):
                self.an_control.publish(target_pose)
            while True:
                if (abs(self.an_current_z - data[2]) < 30):
                    self.wait_time = 0
                    break
                else:
                    self.wait_time += 1
                    time.sleep(1.0)
                if self.wait_time > 15:
                    self.wait_time = 0
                    break
            return True
        else:  # x,y为0则单独控制z
            move_z = [self.an_current_x, self.an_current_y, data[2]]
            target_pose = Float32MultiArray(data=move_z)
            for i in range(4):
                self.an_control.publish(target_pose)
            while True:
                if (abs(self.an_current_z - data[2]) < 30):
                    time.sleep(2.0)
                    break
            return True

    def get_an_control(self, data):
        self.an_grasp_state = data.data

    def set_an_grasp_control(self, data):
        #for i in range(3):
        self.an_grasp_control.publish(data)
        time.sleep(0.2)

if __name__ == '__main__':
    an_start = AN_Mission_Test()
    rospy.spin()
