# 导航功能包集

---

## 配置并使用导航功能包集

### 1.为机器人配置TF

### 2.在ROS上发布传感器数据流


- 目前导航功能包集只接收使用LaserScan或PointCloud两种消息类型发布的传感器数据。  
- 关于ROS消息头（Header）  
    - 字段seq对应一个标识符，随着消息被发布，它会自动增加。
    - 字段stamp存储与数据相关联的时间信息。
    - frame_id:存储坐标名称（与tf相关的信息）。


### 3.通过ROS发布里程计信息


- 导航包使用tf来确定机器人在地图中的位置和建立传感器数据与静态地图的联系，然而TF不能提供任何关于机器人速度的信息 ，所以导航包要求任何的里程计源能通过ROS 发布TF变换和 包含速度信息的nav_msgs/Odometry类型的消息。
- ROS提供了一个消息类型nav_msgs/Odometry来保存机器人内部的位置数据（码盘数据）。该消息的形式为：

```
std_msgs/Header header  #消息头
  uint32 seq
  time stamp
  string frame_id
string child_frame_id  # 子坐标系，一般为/base_link或/base_footprint,表示现实中的机器人
geometry_msgs/PoseWithCovariance pose  #机器人的位置和方向，表示的是机器人“觉得”它相对于开始位置的坐标
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
  float64[36] covariance  # 协方差矩阵，补充了测量的不确定性
geometry_msgs/TwistWithCovariance twist  # 机器人的线速度和角速度
  geometry_msgs/Twist twist
    geometry_msgs/Vector3 linear
      float64 x
      float64 y
      float64 z
    geometry_msgs/Vector3 angular
      float64 x
      float64 y
      float64 z
  float64[36] covariance
```

- 可以通过监听/odom到/base_link(或/base_footprint)之间的变换来监视机器人的位置和方向。

### 4.在机器人上配置并使用导航功能包集


- 导航功能包集假定它可以通过话题“/cmd_vel”来发布geometry_msgs/Twist类型的消息，此消息基于机器人的基坐标系，传递的是运动命令。
- geometry_msgs/Twist消息形式为：

```
geometry_msgs/Vector3 linear  # 线速度
  float64 x
  float64 y
  float64 z
geometry_msgs/Vector3 angular  # 角速度
  float64 x
  float64 y
  float64 z

```

- 因此，在基控制器（base controller）中必须有一个节点订阅“/cmd_vel”话题，并将该话题上速度命令(vx, vy, vtheta转换为电机命令(cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z)发送给移动基座。
 
- 地图（map_server）

- 配置代价地图（local_costmap）& (global_costmap)

    - 导航功能包集需要两个地图用来保存世界中的障碍信息。global_costmap用于全局路径规划，在整个环境中创建长期的路径规划；local_costmap用于局部路径规划与避障。
    - 有一些配置参数两张地图都需要，也有一些各不相同。对于代价地图，有三个配置项：common配置项、global配置项和local配置项。（在costmap_2d文档中有详细介绍）
    - costmap_common_params.yaml：载入时需要在"global_costmap"和“local_costmap”两个命名空间中分别载入    
    - global_costmap_params.yaml
    - base_local_planner_params.yaml
 




























