### 生成deb功能包
在功能包目录下，运行下面指令：
```
bloom-generate rosdebian --os-name ubuntu --ros-distro melodic

fakeroot debian/rules binary
```
### 使用教程
```bash
roslaunch ad_palletizing AD_Palletizing.launch

roslaunch ad_palletizing ZY_AD.launch
```

### 功能包
1. client_ros

通过socket发送照片到中控系统

2. AN_Vision 

识别AR码位姿并通过话题和matt发送出去

3. AN_Mission 

通过识别结果控制岸吊抓取集装箱和放置集装箱

4. test

研发测试程序






### 说明
0x00： 坐标控制
0x01： 当前位置
0x02： 开始
0x03： 暂停
0x04： 回原点
0x05： 吸
0x06： 放
0x07： 复位x
0x08： 复位y

a: x-
d: x+
w: y+
s: y-
q: z+
e: z-
f: 吸/放
1： x复位
2： y复位
3： 退出键盘控制

/Pall_Running_Topic： 岸吊开始（1）和暂停（0）控制话题

/Pall_Grasp_Topic： 爪的吸（1）和放（0）控制

/Pall_Reset_Topic： 复位（1）

/Pall_POS_SET： 坐标控制[x,y,z,u]

/Pall_GRAB_STATUS: 发布抓取状态

/Pall_CURR_POS: 发布当前位姿话题

