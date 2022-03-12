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
