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
    
- 注意事项

    - 对于local_costmap而言，local_planner要求的实时性较高，而局部代价地图所依赖的坐标系一般是odom，绘制local_costmap时需要反复获取odom->base_link的坐标变换，
    tf数据的延迟会影响到costmap，进而影响planner的规划，使得机器人移动迟缓或撞上障碍物。所以要注意参数transform_tolerance的设置。
    
    - 如果使用静态地图做导航，则可以把全局的costmap选择使用static_map选项，这样会节省一些计算量
    
    - 如果采用动态地图（实时slam得到的）或根本不使用先验静态地图，则可以将全局的costmap所依赖的坐标系也改为odom，用rolling_window选项
    替代static选项，这样costmap则会实时更新。此时要注意，上层程序给出的目标点不能超过rolling_window的范围。
 
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
geometry_msgs/Pose origin  # 地图的原点，即地图中的（0,0）单元格在世界坐标系中的位姿（m,m,rad)
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

### 7. 关于map_msgs

#### 消息类型

- OccupancyGridUpdates  
```
Header header
int32 x  # 要更新的部分地图的原点（右下角）在map坐标系下的坐标
int32 y
uint32 width  # 要更新的部分地图的宽和高，单位是cell
uint32 height  
int8[] data  # 更新的地图代价数据
```

- PointCloud2Update
```
uint32 ADD=0
uint32 DELETE=1
Header header
uint32 type          # type of update, one of ADD or DELETE
sensor_msgs/PointCloud2 points
```

- ProjectedMap
```
nav_msgs/OccupancyGrid map
float64 min_z
float64 max_z
```
- ProjectedMapInfo
```
string frame_id
float64 x
float64 y
float64 width
float64 height
float64 min_z
float64 max_z
```

#### 服务类型

- GetMapROI

- GetPointMap

- GetPointMapROI

- ProjectedMapsInfo

- SaveMap

- SetMapProjection

### 7.关于坐标系的关系

- world：世界坐标系，在rviz中是grid的中心点，cell的大小表示每格代表的实际长度（单位是m）。

- map：地图坐标系，一般将其设为固定坐标系(fixed frame)，地图yaml文件中的origin指的地图原点（左下角的点）在世界坐标系中的位置。

- odom：里程计坐标系，从odom-->base_link之间的坐标变换，提供了机器人码盘内部的测量信息（位姿和速度）。
    - 与/odom话题的区分：/odom话题上发布的数据仅仅是轮子编码器的数据，有时候如果机器人还有陀螺仪等传感器（可以对机器人的旋转进行额外的估算，与
    编码器数据合并处理，得到更加精确的位姿数据），则该话题上接收的数据将不是全部的内部测量数据。所以监听odom坐标系到base_link坐标系之间的坐标变
    换比只依赖/odom话题更安全。
    
    - 关于odom和map之间的关系：在机器人的运动开始时，odom坐标系与map坐标系是重合的，但随着时间的推移，里程计会产生累积误差，两个坐标系也会出现
    偏差，map->odom的tf变换就是这个偏差。我们可以通过一些传感器（比如雷达）和package（gmapping等）合作校正获得一个位置估计，据此可以得到
    map-->base_link的tf变换。而这个位置估计和里程计位置估计之间的偏差就是odom和map之间的偏差了。

- base_link：机器人本体坐标系。


### 8.关于map_server

map_server包提供了一个map_server节点，它通过ROS服务的方式提供地图数据。它还提供了map_saver命令行工具用于存储动态生成的地图。

#### 地图格式

- Image格式  
image文件用相应像素的颜色描述了地图上每个单元格的占用状态。在标准配置中，白色是未占用的，黑色是被占用的，灰色是未知的。可以使用彩色图像，但它们会被（平均）转化为灰度值。  一般来说，大多数流行的文件格式都是支持的，但是在OS X系统上PNG格式不被支持。

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

#### 简介

该包提供了利用传感器数据实现2D代价地图的方法，可利用它建立一个2D或3D栅格数据，基于占用单元格或用户指定膨胀潘静的膨胀代价。此外，也支持基于map_server的代价地图初始化、支持基于rolling window的代价地图和基于参数的传感器topic的订阅和配置。

costmap_2d提供了一个可配置的结构，它可以维护关于机器人应该导航到哪里的信息（用占用网格的形式表示）。代价地图使用来自来自静态地图的传感器数据和信息，通过costmap_2d::Costmap2DROS对象来存储和更新障碍物信息。

costmap_2d::Costmap2DROS对象为用户提供了一个纯二维接口，这意味着关于障碍物的查询只能在列中进行。例如，二维平面中有一只表和一只鞋位于相同的位置，但它们的z坐标不同，这也会导致costmap_2d::Costmap2DROS对象的costmap有相同的代价值。这种设计是为了便于在平面空间进行规划。

在Hydro版本中，用于向costmap写入数据的底层方法是完全可配置的，每个功能都分布在不同的层中。例如，静态地图是一层，障碍是另一层。默认情况下，障碍层以三维方式维护信息（参加voxel_grid），三维障碍数据使得该层能更加智能的标记和清除障碍物。

主要的接口是cost_map::Costmap2DROS，它维护了大部分和ROS相关的功能。它包含一个costmap_2d::LayeredCostmap，其被用于跟踪每个层。每一层都可以使用pluginlib在Cosmap2DROS中实例化，并添加到LayeredCostmap中。每一层本身可以被单独编译，允许通过c++接口对costmap进行任意修改。cosmap_2d::Costmap2D类实现了基础数据结构的存储和访问。

#### 一些概念

- 标记和清除  
代价地图通过ROS自动订阅了传感器topics，且可以进行自我更新。传感器被用于标记（向代价地图中插入障碍信息）和清除（从costmap中删除障碍信息），或者两者都有。  
标记操作只是将通过索引数组修改单元格的代价值。  
然而，清除操作由传感器的原点向外的网格进行射线追踪，得到每一个观察结果。如果使用三维结构来存储障碍物信息，则在将每列的障碍物信息放入到代价地图时，会首先被投影到二维平面。

- Occupied、free和unknown空间  
虽然代价地图中的每个单元格可以有255个不同的代价值，但底层结构中只能表示3个。也就是说，每个单元格只有占据、空闲和未知三种状态。每个状态都有一个代价值，分配给它在代价地图上的投影。  
具有一定数量的已占用单元格的列会被赋给一个costmap_2d::LETHAL_OBSTACEL代价；而具有一定数量的未知单元格的列会被赋给一个costmap_2d::NO_INFORNATION代价；其他列则被赋值为costmap_2d::FREE_SPACE代价。

- 地图更新  
代价地图以update_freqency参数指定的频率进行更新。每个周期内，传感器数据的接收、障碍物的标记和清除操作都在代价地图的底层结构中完成，这个结构被投影到代价地图中，并在地图中为每个单元格分配适当的代价值。  
此后，利用costmap_2d::LETHAL_OBSTACEL代价值为每个单元格执行膨胀，这包括从被占用的单元格开始向外传递代价值至用户指定的膨胀半径。

- tf  
为了将来自传感器的数据插入到代价地图中，cosmap_2d::Costmap2DROS对象普遍使用tf变换。具体来讲，它假定由global_frame参数、robot_base_frame参数和传感器指定的坐标系之间的变换都是相互连接的、且是最新的。tramsform_tolerance参数设定的是这些坐标转换允许的最大延迟。如果tf树没有按照该预期速度更新，导航栈就会停止机器人。

- 膨胀：  
膨胀是指从被占据的单元格开始向外传递代价值（随距离递减？）的过程。这里，我们为与机器人相关的代价值定义了5个符号：

    - "Lethal"代价意味着工作空间中存在实际的障碍物，因此当机器人的中心在这个单元格中时，机器人一定会发生明显的碰撞。
    
    - "Inscribed"代价意味着单元格小于机器人的内切半径和实际障碍物之间的距离。所以，如果机器人的中心在单元格内，机器人会发生碰撞。
    
    - "Possibly circumscribed"代价类似于"Inscribed"，但是它使用机器人的外接圆半径作为边界距离。因此，如果机器人的中心位于等于或高于这个值
    的单元格内，则机器人是否会与障碍物发生碰撞取决于它的方向。说possiblly，是因为它可能不是真正的障碍单元格，而是一些用户的偏好，他们在地图中
    加入了这个代价值。例如，如果用户想要表示机器人应该试图躲避建筑物的某一块特定的区域，他们可以在代价地图中设定自己的代价值而不依赖于任何障碍物。
    
    - "Freespace"代价被认为是0，它意味着该单元格没有障碍物。
    
    - "Unknown"代价意味着没有关于该单元格的信息。
    
    - 其他的位于"Freespace"和"Possibly circumscribed"之间的代价值取决于他们距离"Lethal"单元格的距离和用户提供的衰减函数。
    
- 地图类型  
初始化一个costmap_2d::Costmap2DROS对象的方法有两种。
  
    - 第一种是使用用户生成的静态地图，这种情况下，代价地图被初始化为和用户提供的静态地图宽、高、障碍物信息相匹配。这种配置通常用于本地系统，如
    amcl，它允许机器人在地图坐标系中注册障碍信息且可以在行动中利用传感器信息更新代价地图。
    
    - 第二种初始化的方法是给它一个宽和高，同时设置rolling_window参数为True。rolling_window参数使得机器人在移动过程中，保持在代价地图的中心，
    且在机器人离给定区域过远时，从地图中丢弃障碍物信息。这种类型的配置最常用于odometric坐标系，此时机器人仅关心附近区域的障碍物。

#### API

- Costmap2DROS

- 分层地图

    - 静态地图层（Static Map Layer）  
    静态地图层，通过slam算法生成的地图，不可改变，不包括动态或静态障碍
    
    - 障碍物地图层（Obstacle Map Layer）  
    障碍地图层，通过传感器数据识别出非固定地图的障碍，包括动态，静态障碍
    
    - 膨胀层（Inflation Layer）  
    膨胀层，根据inflation半径计算出的robot所占的可能最大面积，可选的
    
    - 其他层（Other Layer）  
    可以通过插件增加其他的层结构  

    - 参考链接
        - 链接1：[详细介绍了costmap_2d](https://blog.csdn.net/jinking01/article/details/79455962)
        - 链接2：[导航包介绍](http://blog.exbot.net/archives/1129)
 
- 新建一个地图层

    - 参考链接1: [wiki教程](http://wiki.ros.org/costmap_2d/Tutorials)

    - 参考链接2: [博客教程（基于catkin）](https://blog.csdn.net/heyijia0327/article/details/42241831)
    
    - 参考链接3：[博客教程2](https://blog.csdn.net/x_r_su/article/details/53454368)
        
    
### 10.关于导航包发布的一些话题

- /amcl_pose
    
    - 发布者：amcl
    
    - 消息类型：geometry_msgs/PoseWithCovarianceStamped
    
    - 消息内容：机器人在世界坐标系下的位姿。
    
- /map

    - 发布者：map_server
    
    - 消息类型：nav_msgs/OccupancyGrid
    
    - 消息内容：静态地图信息

- /move_base/local_costmap/costmap

    - 发布者：move_base
    
    - 消息类型：nav_msgs/OccupancyGrid
    
    - 消息内容：局部代价地图，和全局代价地图的width、height、resolution均一致，但其考虑的是机器人周围区域的障碍物信息。
    
- /move_base/local_costmap/costmap_updates

    - 发布者：move_base
    
    - 消息类型：map_msgs/OccupancyGridUpdate
    
    - 消息内容：要更新的局部地图信息，(x,y)是局部地图原点（右下角）在map坐标系下的坐标
    
- /move_base/global_costmap/costmap

    - 发布者：move_base
    
    - 消息类型：nav_msgs/OccupancyGrid
    
    - 消息内容：基于静态地图和局部地图信息，生成的全局代价地图（会随时间更新）
    
- /move_base/global_costmap/costmap_updates

    - 发布者：move_base
    
    - 消息类型：map_msgs/OccupancyGridUpdate
    
    - 消息内容：要更新的全局代价地图信息（根据local costmap进行局部更新）
    
- /move_base/TrajectoryPlannerROS/global_plan

    - 发布者：move_base
    
    - 消息类型：nav_msgs/Path
    
    - 消息内容：全局规划器规划出的全局路径

- /move_base/TrajectoryPlannerROS/local_plan

    - 发布者：move_base
    
    - 消息类型：nav_msgs/Path
    
    - 消息内容：局部规划器规划出的局部路径





















