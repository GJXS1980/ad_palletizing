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
