# parrot2无人机yolov3目标物体追踪

- yolov3目标识别
- 物体追踪
- 无目标搜索（施工中）

## Overview

- [MRZIRC比赛](http://www.mbzirc.com/challenge/2020)的第一个竞赛项目是无人机在100X60m的区域下寻找有悬挂气球的无人机，并且捕获目标气球，学校校内赛是在50mX60m的范围内寻找目标气球并且刺破；
- [ROS](www.ros.org)是一款为机器人设计的系统框架，[parrot2](http://www.parrot.com.cn/)无人机提供ROS的开发通信环境[bebop_autonomy](http://wiki.ros.org/bebop_autonomy)；
- [darknet yolov3](https://pjreddie.com/darknet/yolo/)是深度学习的目标检测框架，C++/C编写；
- 这个项目是基于一位外国的github的项目修改的[darknet_ros](https://github.com/leggedrobotics/darknet_ros)项目。

## Prepare
- 笔记本配置：i5，GTX1060
- 软件环境：Ubuntu16.04、 kinetic版本ROS

## Vidio
- [YouTube](https://youtu.be/F69QrPSHUOs)

## How to used
### build
```
cd catkin_ws/src
git clone https://github.com/AutonomyLab/bebop_autonomy.git
git clone https://github.com/ShiSanChuan/darknet_ros.git
cd ..
catkin build bebop_autonomy
catkin build darknet_ros -DCMAKE_BUILD_TYPE=Release
<!--  -->
source devel/setup.bash
```
### Download weights
- 原COCO data 训练好的框架
```
wget http://pjreddie.com/media/files/yolov3-voc.weights
wget http://pjreddie.com/media/files/yolov3.weights
```
- 比赛训练的模型框架文件(待传)

### run
- 参照bebop_autonomy的链接，打开parrot2后笔记本连接parrot的网络，运行
```
roslaunch bebop_driver bebop_node.launch
```
- 运行成功后运行darknet ros，启动后飞机会自动起飞，注意安全，若不希望起飞在代码中修改下
```
roslaunch darknet_ros yolo_v3.launch
```
- 停止运行，在terminal输入
```
rostopic pub --once /bebop/land std_msgs/Empty
```

## Thinking
- 通过深度学习框架比如yolo来识别物体是可以的，不过受到图片尺寸问题，对于近处目标（大概3米）的目标可以识别，但对于远处目标（在parrot传来的428x240图片里大概占5个像素），这种目标是无法分辨出来的，因此在原基础代码上添加opencv的颜色识别的线程，用于辅助yolo的识别，使得无人机在比较远处能先靠近物体，若不是该目标物体再移动视角环境，找下一个疑似目标；
- 由于目标是移动的或者静止的，因此通过PID调整飞行参数使得目标的中心位置与图像中心位置重合。

## Parament
- 安装原darknet_ros程序步骤，需要将训练模型的weight和cfg文件配置好，比赛识别的目标是气球因此专门为气球识别训练了几组weight
```
catkin_wc/src/darknet_ros/darknet_ros/yolo_network_config/weights/
catkin_wc/src/darknet_ros/darknet_ros/yolo_network_config/cfg/
```
- 在launch文件里修改launch启动文件,yaml文件设置识别阈值和目标类型

```
catkin_wc/src/darknet_ros/darknet_ros/launch/yolo_v3.launch

<rosparam command="load" ns="darknet_ros" file="$(find darknet_ros)/config/yolov3-MBZIRC.yaml"/>
```

## used on other UAV
- 若要使得该无人机目标追踪能在其他平台上使用（比如M100），除了处理修改自己要的识别目标的模型文件外，还必须按照bebop的话题修改ROStopic，同时对于无人机的控制命令（起飞、降落、姿态调整）的rostopic保持一致；

```
<param name="bebop_topic_head"  value="/bebop" />
```
- 若要修改识别目标，先按照darknet官网训练目标的模型文件，在调节下远处目标识别的大概颜色。

## Contributier
- 队友[Voyager](https://github.com/VoyagerIII)
- M. Bjelonic
**"YOLO ROS: Real-Time Object Detection for ROS"**,
URL: https://github.com/leggedrobotics/darknet_ros, 2018.

    @misc{bjelonicYolo2018,
      author = {Marko Bjelonic},
      title = {{YOLO ROS}: Real-Time Object Detection for {ROS}},
      howpublished = {\url{https://github.com/leggedrobotics/darknet_ros}},
      year = {2016--2018},
}
