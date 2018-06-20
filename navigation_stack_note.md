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

### 6.关于nav_msgs中的消息和服务

#### 消息类型

- GridCells 栅格单元 
```
std_msgs/Header header  # 消息头，时间戳和坐标系
float32 cell_width  # 单元格的宽 
float32 cell_height  # 高
geometry_msgs/Point[] cells  # 单元格数组，存储单元格位置信息？
```

- MapMetaData  地图基本信息
```
time map_load_time  # 地图被加载的时间
float32 resolution  # 地图分辨率（m/cell）
uint32 width  # 地图的宽，单位是cell?
uint32 height  # 地图的高
geometry_msgs/Pose origin  # 地图的原点，即地图中的（0,0）单元格在真实世界中的位姿（m,m,rad)
```

- OccupancyGrid  2d栅格地图，其中每个单元格表示占用的概率
```
std_msgs/Header header
MapMetaData info  # 地图的基本信息
int8[] data  # 地图数据，依行排序，从(0,0)点开始。占用率的取值范围是[0,100]，未知为-1。
```

- Odometry  里程计（轮子编码器）消息，表示机器人内部的位姿和速度测量（估计）信息
```
std_msgs/Header header
string child_frame_id  # 一般设为base_link或base_footprint
geometry_msgs/PoseWithCovariance pose  # 位姿信息
geometry_msgs/TwistWithCovariance twist  # 速度信息
```

- Path  用一个pose序列表示的路径信息
```
std/Header header
geometry_msgs/PoseStamped[] poses  # 存储pose的数组
```

#### 服务类型

- GetMap  接收地图
```
 # 不发送数据(调用服务无需发送数据)
 ---
 # 接收数据（调用后返回的数据）
 nav_msgs/OccupancyGrid map
```

- GetPlan  得到一个从当前位置到目标位姿的路径规划
```
 # 发送数据
 geometry_msgs/PoseStamped start  # 初始位姿
 geometry_msgs/PoseStamped goal  # 目标位姿
 float32 tolerance  # 如果不能到达目标，在失败前，规划器可以在x和y方向上放松的约束条件（m）（就近选取目标？）
 ---
 # 接收数据
 nav_msgs/Path plan  # 规划的路径
```

- SetMap  # 创建一个带有初始位姿的新地图
```
 # 发送数据
 nav_msgs/OccupancyGrid map
 geometry_msgs/PoseWithCovarianceStamped initial_pose
 ---
 # 接收数据
 bool success
```

#### action类型

- GetMap.action
```
  # 定义一个目标，这里没有
  ---
  # define the result
  nav_msgs/OccupancyGrid map
  ---
  # 没有feedback消息
```

### 7.关于坐标系的关系

- map：地图坐标系，一般将其设为固定坐标系(fixed frame)，与机器人所在的世界坐标系一致

- odom：里程计坐标系，从odom-->base_link之间的坐标变换，提供了机器人码盘内部的测量信息（位姿和速度）。
    - 与/odom话题的区分：/odom话题上发布的数据仅仅是轮子编码器的数据，有时候如果机器人还有陀螺仪等传感器（可以对机器人的旋转进行额外的估算，与编码器数据合并处理，得到更加精确的位姿数据），则该话题上接收的数据将不是全部的内部测量数据。所以监听odom坐标系到base_link坐标系之间的坐标变换比只依赖/odom话题更安全。
    - 关于odom和map之间的关系：在机器人的运动开始时，odom坐标系与map坐标系是重合的，但随着时间的推移，里程计会产生累积误差，两个坐标系也会出现偏差，map->odom的tf变换就是这个偏差。我们可以通过一些传感器（比如雷达）和package（gmapping等）合作校正获得一个位置估计，据此可以得到map-->base_link的tf变换。而这个位置估计和里程计位置估计之间的偏差就是odom和map之间的偏差了。

- base_link：机器人本体坐标系。


### 8.关于map_server

map_server包提供了一个map_server节点，它通过ROS服务的方式提供地图数据。它还提供了map_saver命令行工具用于存储动态生成的地图。

#### 地图格式

- Image格式

image文件用相应像素的颜色描述了地图上每个单元格的占用状态。在标准配置中，白色是未占用的，黑色是被占用的，灰色是未知的。可以使用彩色图像，但它们会被（平均）转化为灰度值。

一般来说，大多数流行的文件格式都是支持的，但是在OS X系统上PNG格式不被支持。

- Yaml文件

yaml文件描述了地图的维度，其格式如下：

```
image: testmap.png  # 包含占有率数据的图形文件的路径，可以是绝对路径，也可以是相对路径（相对yaml文件）
resolution: 0.1  # 地图的分辨率，m/pixel
origin: [0.0, 0.0, 0.0]  # 地图左下像素的二维坐标，表示为（x,y,yaw），yaw为逆时针旋转的角度。（大多数系统会忽略旋转角）
occupied_thresh: 0.65  # 占用率大于该值的，被认为是完全占据
free_thresh: 0.196  # 占用率小于该值的，被认为是空闲
negate: 0  # 是否将 白/黑 free/occupied 反向表示

可选参数
mode: trinary/scale/raw  # 默认值为trinary
```

- 给一个在[0,256]之间的灰度值x，图像单元的占用率的计算方法为：p = (255-x)/255（如果negate=0），这意味着黑色（灰度值为0）的占用率为1
- trinary mode的解释：当p>occupied_thresh，则输出值为100，当p<free_thresh，则输出值为0，否则输出-1（表明该cell为未知）。
- scale mode的解释：当p>occupied_thresh，则输出值为100，当p<free_thresh，则输出值为0，否则输出99*(p-free_thresh)/(occupied_thresh-free_thresh)，这将允许输出[0,100]范围内的完整梯度值。
- raw mode的解释：直接输出x的值，所以输出范围为[0,255]。

#### 命令行工具

**map_server**

- map_server 是一个ROS节点，可用来读取一张地图并将它提供给ROS service。
```
    rosrun map_server map_server
```
运行后，可以通过指定的topic来获取地图数据（意味着它可以发送给新的订阅者），也可以通过服务获取。

- 发布的话题

/map_metadata(nav_msgs/MapMetaData):可以通过这个话题接收地图元数据

/map(Nav_msgs/OccupancyGrid)：可以通过此话题来接收地图数据

- 服务

static_map(Nav_msgs/OccupancyGrid)：可以通过此服务来接收地图


**map_saver**

- 用于存储一张来自SLAM建图service的地图。

```
    rosrun map_server map_saver -f mymap
```
- 订阅的话题

map(nav_msgs/OccupancyGrid)

### 9.关于costmap_2d
























