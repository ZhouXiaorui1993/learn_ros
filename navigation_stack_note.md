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


- 导航包使用tf来确定机器人在地图中的位置和建立传感器数据与静态地图的联系，然而TF不能提供任何关于机器人速度的信息 ，所以导航包要求任何的里程计源能通过ROS发布TF变换和包含速度信息的nav_msgs/Odometry类型的消息。
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
    - 通用配置，costmap_common_params.yaml：包括传感器读取的最大范围、清除障碍物的有效范围、机器人的占用面积（圆形机器人的半径）、障碍物的膨胀半径、传感器源等信息。注意：此文件载入时需要在"global_costmap"和“local_costmap”两个命名空间中分别载入。
    - 全局代价地图配置，global_costmap_params.yaml：包括代价地图应该运行的坐标系（一般设置为/map）、机器人基坐标系、代价地图更新频率以及是否是静态地图等参数。
    - 局部代价地图配置，local_costmap_params.yaml：包括global框架（一般设置为/odom）、机器人基坐标系、局部地图更新频率、代价地图发布可视化信息的频率、地图的宽、高和分辨率（一般设置为和静态地图相同）等信息。
    - 基本本地规划器配置，base_local_planner_params.yaml：包括速度和加速度限制等。
 
### 5.使用Rviz可视化导航过程

- 2D导航目标/2D Nav Goal

    - topic：move_base_simple/goal
    - 消息类型：geometry_msgs/PoseStamped
    - 说明：允许用户设置机器人的目标位姿，并发送到导航包

- 2D姿态估计/2D Pose Estimate
    - 话题：initialpose
    - 消息类型：geometry_msgs/PoseWithCovarianceStamped
    - 说明：初始化机器人位姿

- 静态地图/Static Map
    - 话题：map
    - 服务：nav_msgs/GetMap（调用该服务无需提供参数，接收回应的数据消息类型为`nav_msgs/OccupancyGrid map`类型）
    - 说明：显示由map_server提供的静态地图（如果存在的话）

- 粒子云/Partial Cloud
    - 话题：partialcloud
    - 消息类型：geometry_msgs/PoseArray
    - 说明：显示由机器人定位系统产生的粒子云，粒子云的分布代表了定位系统对机器人位姿的估计的准确性。粒子云越分散，表示不确定性越高。

- 机器人本体/Robot Footprint
    - 话题：local_costmap/robot_footprint
    - 消息类型：geometry_msgs/PolygonStamped
    - 说明：显示机器人占用的空间

- 障碍/Obstacles
    - 话题：local_costmap/obstacles
    - 消息类型：nav_msgs/GridCells
    - 说明：显示本地代价地图中的障碍物。为避免机器人碰撞，机器人本体不应与障碍物所在的单元有交集。

- 障碍膨胀区/Inflated Obstacles
    - 话题：local_costmap/inflated_obstacles
    - 消息类型：nav_msgs/GridCells
    - 说明：在代价地图中显示障碍物的膨胀区（由机器人的内切半径r决定，因为机器人中心到达距离障碍物r远的位置时，实际上边缘部分已经与障碍物发生了碰撞）。为避免发生碰撞，机器人的中心一定不能与障碍膨胀区有交集。

- 未知空间/Unknown Space
    - 话题：local_costmap/unknown_space
    - 消息类型：nav_msgs/GridCells
    - 说明：显示导航包costmap_2d中包含的未知空间

- 全局规划/Global Plan
    - 话题：TrajectoryPlannerROS/global_plan
    - 消息类型：nav_msgs/Path
    - 说明：显示到达目标的全局规划路径（本地规划local_planner当前追踪的路径）

- 本地规划/Local Plan
    - 话题：TrajectoryPlannerROS/local_plan
    - 消息类型：nav_msgs/Path
    - 本地规划路径（与发送给基控制器的速度指令相关）

- 规划器规划/Planner Plan
    - 话题：NavfnROS/plan
    - 消息类型：nav_msgs/Path
    - 说明：由全局规划器（navfn或global_planner）计算出的机器人的完整规划

- 当前目标/Current Goal
    - 话题：current_goal
    - 消息类型：geometry_msgs/PoseStamped
    - 说明：显示导航包视图到达的目标方位

### 


























