* this is modification of ares_multirobots and aws-robomaker-hospital-world, which are modified for multirobot system simulation, including Formation Control, VINS, Sensor Fusion and so on. detialed of original projects see the reference.  

# Start up
```bash
 # load world and three robots
 roslaunch ares_gazebo world_robots_gazebo.launch 
 # control command motivate car // can be written into launch
 rosrun ares_teleop ares_teleop.py __ns:=ares1 
 
```


## Reference
### multirobot_formation
car models and simple control nodes are reference here.
[github](https://github.com/guyuehome/multirobot_formation)

基于ros的多机器人仿真（导航+编队）
详细请参考项目笔记（包含导航、编队原理介绍，详细配置过程）：
1、《如何在Gazebo中实现多机器人仿真》 ： https://www.guyuehome.com/4889
2、《基于ROS的多机器人编队仿真》 ： https://www.guyuehome.com/8907
3、《如何在Rviz中实现多机器人导航仿真》 ： https://www.guyuehome.com/8912
4、《基于ROS的多机器人导航+编队仿真》： https://www.guyuehome.com/8915


### aws-robomaker-hospital-world
simulation world is reference here.
[github](https://github.com/aws-robotics/aws-robomaker-hospital-world)
* you need download gazebo official models lib and referenced models detailed in this project's file 'setup.py'. Also most likely you need to fix the url problem arised when loading world. 

### XTDrone
UAV Simulation Platform based on PX4, ROS and Gazebo
[github](https://github.com/robin-shaun/xtdrone)
