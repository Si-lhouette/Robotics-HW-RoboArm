## ReadMe

### File

| Folder      | Presentation                                                 |
| ----------- | ------------------------------------------------------------ |
| /code       | 含`probot_gazebo` ROS Package; 其中/src 下包含实验1-5&大作业代码 |
| /img        | 文档内所需图片                                               |
| /video      | 验证视频                                                     |
| /doc        | md文档                                                       |
| /experiment | 机械臂敲铃实物代码及报告                                     |

实物敲铃实验的代码为MATLAB代码，使用了MATLAB ROS ToolBox.



### Run

```
catkin_make
```

##### Lab 1 正逆运动学

```
roslaunch probot_gazebo probot_anno_position_control_bringup.launch 
roslaunch probot_gazebo invert.launch 
```

##### Lab 3 Jacobi速度传递

```
roslaunch probot_gazebo probot_anno_velocity_control_ring_bringup.launch 
roslaunch probot_gazebo jacobi.launch 
```

##### Lab 4 轨迹规划

```
roslaunch probot_gazebo probot_anno_velocity_control_ring_bringup.launch 
roslaunch probot_gazebo trajectory.launch 
```

##### Lab 5 定点转动

```
roslaunch probot_gazebo probot_anno_velocity_control_ring_bringup.launch 
roslaunch probot_gazebo fixpoint.launch 
```

##### 大作业 Ring

```
roslaunch probot_gazebo probot_anno_velocity_control_ring_bringup.launch 
roslaunch probot_gazebo ring.launch 
```



### Depends

Eigen 3.3.7