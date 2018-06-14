# ROS系统架构

ROS系统的架构主要被划分为三部分，每个部分都代表一个层级的概念:

- 文件系统级(filesystem level)
- 计算图级(computation graph level)
- 开源社区级(community level)

## 文件系统级

- **功能包(Package)**:ROS中软件组织的基本形式，一个功能包具有最小的结构和最少的内容，用于创建ros程序。
- **功能包清单(Manifest)**:提供关于功能包、许可信息、依赖关系、编译标志等信息。它是一个`manifests.xml`文件，通过这个文件可以实现对功能包的管理。
- **功能包集(Stack)**:将几个具有某些功能的功能包组织在一起得到的。在ros系统中，存在大量的不同用途的功能包集，如导航功能包集。
- **功能包集清单(Stack manifest)**:是一个`stack.xml`文件，作用类似功能包清单。
- **消息类型(Message /msg type)**:消息是一个进程发送到其他进程的信息。ros中有很多标准类型消息，消息类型的说明存储在`my_package/msg/MyMessageType.msg`中，也就是对应功能包的msg文件夹下。
- **服务类型(Service/srv type)**:对服务的类型进行描述说明的文件，存储在对应功能包的srv文件夹下。

###功能包

功能包指的是一种特定的文件结构和文件夹组合，如下所示:  

- `bin/` 编译和链接程序后，用于存储可执行文件的文件夹。
- `include/package_name/` 包含了所需要的库的头文件。
- `msg/` 如果要开发非标准消息，请把文件放在这里。
- `scripts/` 包括Bash、Python或任何其他可执行脚本的可执行脚本文件。
- `src/` 存储程序源文件的地方。
- `srv/` 存储描述服务类型的文件。
- `CMakeLists.txt` 这是CMake的生成文件。
- `package.xml`(catkin build生成)或`manifest.xml`(rosbuild生成) 功能包清单文件:  
    
    - 必须包含在功能包中
    - 在该文件中有两个典型的标记:`<depend>`和`<export>`，前者会显示当前功能包安装之前必须先安装哪些功能包；后者会告诉系统编译该功能包需要使用什么编译标志，引用哪些头文件等。  

- `launch/` 存储`.launch`文件，在ROS应用中，每个节点通常都有许多参数需要设置，为方便高效操作多个节点，可以编写launch文件，然后用`roslaunch`命令运行。
例如：  
```html
<launch>
<node pkg="beginner_tutorials" type="example" name="example1" output="screen"/>
</launch>
```  
在上面的配置中，功能包名传递给pkg属性，节点名传递给type属性。由于同一个节点可以运行多个实例，因此需要给这个节点的实例起一个名称，传递给name属性。
同时，最好将这些属性配置输出到屏幕，这样就会在与启动节点相同的shell窗口中出现相应的调试信息。如果想要在节点启动时调用GDB调试器，需要添加
`launch-prefix="xterm -e gdb --args"`，请注意此时会打开一个新的窗口，作为调试窗口。  

为创建、修改或使用功能包，ROS给我们提供了一些工具:  

- `rospack` 使用此命令来获取信息或在系统中查找功能包。   

    - `rospack find [package_name]`:
    - `xxx`

- `roscreate-pkg [package_name] [depend1] [depend2] [depend3]`: 使用此命令创建一个新的功能包。  
- `rosmake` 使用此命令来编译功能包。
- `rosdep` 使用此命令安装功能包的系统依赖项。
- `rxdep` 查看功能包的依赖关系图。

若要在文件夹和功能包之间移动文件，ROS提供了非常有用的rosbash功能包，其中包含了一些类似linux命令的命令，例如:  

- `roscd [package_name]` 用于更改目录到某个功能包。
- `rosed [package_name] [filename]` 用于编辑文件，利用它可以直接通过package名来获取待编辑的文件，而无需指定该文件的存储路径。它默认的编辑器是Vim，如果想要设置其他编辑器为默认，需要修改`~/.bashrc`文件，增加语句如:`export EDITOR='emacs -nw'`，则将emacs设置成为默认编辑器。**注意**:`.bashrc`文件的改变，只会在新的终端才有效。已经打开的终端不受环境变量的影响。  
- `roscp` 从一些功能包中复制文件。
- `rosd [package_name]`: 列出功能包的目录。
- `rosls [package_name]`: 列出功能包/功能包集下的文件

###功能包集

如果说功能包的目标是易于代码复用的最小代码集合，那么功能包集的目标则是简化代码共享的过程。   

常用命令:  

- `roscreate-stack` 用来创建一个功能包集。  
- `rosstack find '[stack_name]'` 查找系统中已安装的某个功能包集  

功能包集必需的三个文件是:`CMakeList.txt`、`Makefile`和`stack.xml`。  

###消息类型

ROS使用了一种简化的消息类型描述语言来描述ROS节点发布的数据值。通过这样的描述语言，ROS能够使用多种编程语言生成不同类型消息的源代码。  

ROS提供了很多预定义的消息类型。如果你创建了一种自定义消息类型，就要把它的类型定义放在功能包的`msg/`文件夹下。在该文件夹中，有用于定义各种消息的文件，这些文件都以`.msg`为扩展名。  

消息类型必须具有两个主要部分:字段和常量。字段定义了要在消息中传输的数据类型，常量用于定义字段的名称，如:  
```
int32 id
string name
```
ROS消息中的一种特殊数据类型是Header，主要用于添加时间戳、坐标位置等。报文头还允许对消息进行编号。通过报文头内部附加信息，我们可以知道是哪个节点发出的消息，或者可以添加一些能够被ROS处理的其他功能。在msg文件的第一行经常可以看到`Header header`的声明。  

###服务类型

ROS使用了一种简化的服务描述语言来描述ROS的服务类型。服务的描述存储在功能包的`srv/`子目录的`.srv`文件中。  

`.srv`文件分为请求和响应两部分，由"---"分隔。示例如下:  

```
int64 A
int64 B
---
int64 Sum
```
其中，A和B是请求，Sum是响应。  

若要调用服务，需要使用该功能包的名称以及服务名称。例如，对于`sample_package1/srv/sample1.srv`文件，可以将其称为`sample_package1/sample1`服务.  

`rossrv`工具可以输出服务说明、`.srv`文件所在的功能包名称，并可以找到使用某一服务类型的源代码文件。  

如果想要创建一个服务，可以使用服务生成器。这些工具能够从基本的服务说明中生成代码，只需要在`CMakeList.txt`文件中加一行`gensrv()`命令。  

##计算机图级

ROS会创建一个连接到所有进程的网络，在系统的任何节点都可以访问此网络，并通过该网络与其他节点交互，获取其他节点发布的信息，并将自身数据发布到网络上。  

在这一层级中最基本的概念包括节点、节点管理器、参数服务器、消息、服务、主题和消息记录包。  

- **节点(Node)**: 节点是主要的计算执行进程。通常情况下，系统包含能够实现不同功能的多个节点。为方便管理，最好让每一个节点都具有特定的单一的功能，而非创建一个包罗万象的大节点。节点需要使用如roscpp或rospy的ROS客户端库进行编写。  
- **节点管理器(Master)**: 用于节点的名称注册和查找等。如果整个ROS系统中没有节点管理器，就不会有节点、服务、消息之间的通信。需要注意，由于ROS本身是一个分布式网络系统，所以你可以在计算机A上运行节点管理器，在计算机B上运行由该管理器管理的节点。  
- **参数服务器(Parameter Server)**: 参数服务器能够使数据通过关键词存储在系统的核心位置。通过使用参数，就能够在运行时配置节点或改变节点的工作任务。  
- **消息(Message)**: 节点通过消息完成彼此的沟通，消息包含一个节点发送到其他节点的数据信息。
- **主题(Topic)**: 主题是由ROS网络对消息进行路由和消息管理的数据总线。每一条消息都要发布到相应的主题。当一个节点发送数据时，我们就说该节点正在向主题发布消息。节点可以通过订阅投个主题，接收来自其他节点的消息。消息的订阅者和发布者之间是互相解耦的，订阅了某主题不一定要向该主题发布消息。需要注意的是，主题的名称必须是独一无二的，否则在同名主题之间的消息路由就会发生错误。  
- **服务(Service)**: 在发布主题时，正在发送的数据能够以多对多的方式交互。但当你需要从某个节点获得一个请求或应答时，就不能通过主题来实现了。这种情况下，服务能够允许我们直接与某个节点进行交互。此外，服务也必须有一个唯一的名称。当一个节点提供某个服务时，所有的节点都可以通过使用ROS客户端库所编写的代码与它通信。  
- **消息记录包(Bag)**: 是一种用于保存和回访ROS消息数据的文件格式，它是一种存储数据的重要机制。它能够获取并记录各种难以收集的传感器数据，我们可以通过消息记录包来反复获取实验数据，进行必要的开发和算法测试。  

###节点

节点是各自独立的可执行文件，能够通过主题、服务或参数服务器来与其他进程(节点)通信。ROS使用节点来将代码和功能解耦，提高了系统的容错性和可维护性；同时节点允许了ROS系统能够布置在任意多个机器上并同时运行。  

节点在系统中必须有唯一的名称。它可以使用不同的库进行编写，如roscpp和rospy，前者基于C++，后者基于Python。  

ROS提供了处理节点的工具，如`rosnode`。它是一个用于显示节点信息的命令行工具，支持的命令如下:  

- `rosnode info node` 输出当前节点信息
- `rosnode kill node` 结束当前运行节点进程或发送给定信号
- `rosnode list` 列出当前活动的节点
- `rosnode machine hostname` 列出某一特定计算机上运行的节点或列出主机名称
- `rosnode ping node` 测试节点之间的连通性
- `rosnode cleanup` 将无法访问的节点的注册信息清除

启动一个新的节点:  

`rosrun [package_name] [node_name]`  

可以在启动时，更改节点名称:  

`rosrun [package_name] [node_name] _name:=[new_node_name]`  

`roslaunch`可以用来启动定义在launch文件中的多个节点:  

`roslaunch [package] [filename.launch]`  


###主题

主题是节点间用来传输数据的总线，通过主题进行消息路由不需要节点之间直接连接。   

每个主题都是强类型的，发布到主题上的消息必须与主题的ROS消息类型匹配，并且节点只能接收类型匹配的消息。因此，一个节点想要订阅一个主题，就必须和该主题具有相同的消息类型。  

ROS的主题可以使用TCP/IP和UDP传输。基于TCP的传输称为TCPROS，它使用TCP/IP长连接，这是ROS默认的传输方式。  

基于UDP的传输称为UDPROS，它是一种低延迟高效率的传输方式，但可能产生数据丢失，所以它最适合与远程操控任务。  

ROS有一个命令行工具`rostopic`用于主题操作，参数如下:  

- `rostopic bw /topic` 显示主题所使用的带宽。
- `rostopic echo /topic` 可以查看节点发出的信息，消息产生时发送了哪些数据。
- `rostopic find message_type` 按照类型查找主题。
- `rostopic hz /topic` 显示主题的发布率。
- `rostopic info /topic` 输出活动主题、发布的主题、主题订阅者和服务的信息。
- `rostopic list` 输出活动主题的列表。
- `rostopic pub /topic type args` 将数据发布到主题，它允许我们直接从命令行中对任意主题创建和发布数据。
- `rostopic type /topic` 输出主题的类型，或者说主题中发布的消息类型。

###服务

当你需要直接与节点通信并获得应答时，就需要使用服务。  

服务需要由用户开发，节点并不提供标准服。包含消息源代码的文件存储在`srv`文件夹中。  

ROS关于服务的命令行工具有两个: rossrv和rosservice。我们可以通过`rossrv`来看到有关服务数据结构的信息，并且与`rosmsg`具有完全一致的用法。通过`rosservice`可以列出服务列表和查询某个服务:  

- `rosservice call /service args` 根据命令行参数调用服务。
- `rosservice find msg-type` 根据服务类型查询服务。
- `rosservice info /service` 输出服务信息。
- `rosservice list` 输出活动服务清单。
- `rosservice type /service` 输出服务类型，若类型显示为Empty，则表明服务类型为空，调用该服务不需要参数。
- `rosservice uri /service` 输出服务的ROSRPC URI.

###消息

一个节点通过向特定主题发布消息，从而将数据发送到另一个节点。消息具有一定的类型和数据结构，包括ROS提供的标准类型和用户自定义的类型。  

消息类型命名: 功能包名称`/.msg`文件名称。  

命令行工具`rosmsg`，惯用参数如下:  

- `rosmsg show` 显示一条消息的字段。
- `rosmsg list` 列出所有消息。
- `rosmsg package` 列出功能包的所有消息。
- `rosmsg packages` 列出所有具有该消息的功能包。
- `rosmsg users` 搜索使用该消息类型的代码文件。
- `rosmsg md5` 显示一条消息的MD5求和结果。

###消息记录包

消息记录包是由ROS创建的一组文件，使用`.bag`格式保存消息、主题、服务和其他ROS数据消息。  

记录包文件可以像实时会话一样在ROS中再现情景，在相同时间向主题发送相同的数据。通常情况下，我们可以使用此功能来调试算法。  

命令行工具:  

- `rosbag` 用来录制、播放和执行其他操作(本机将录制文件放在了`~/bagfiles`)。

可以将对`rosbag record`的调用直接包含在launch文件中执行，我们需要在launch文件中增加一个节点如下： 
```html
<node pkg="rosbag" type="record" name="bag_record" args="其他命令参数（如 /package_name /topic_name ）"/>
```
需要注意的是，在使用launch文件直接启动记录时，记录包文件会默认创建在`~/.ros`路径下，除非使用`-o`前缀或`-O`全名，给文件命名。

- `rxbag` 用于可视化图形环境中的数据。
- `rostopic` 帮助我们看到节点发送的主题。

###节点管理器

ROS节点管理器向ROS系统中其他节点提供命名和注册服务，它像服务一样跟踪和记录主题的发布者和订阅者。它的作用是使得ROS节点之间能够互相查找。一旦这些节点找到了彼此，就能建立一种点对点的通信方式。  

节点管理器通常使用`roscore`命令运行，它会加载ROS节点管理器和一系列其他ROS核心组件。  

###参数服务器

参数服务器是可通过网络访问的共享的多变量字典，节点使用此服务器来存储和检索运行中的参数。  

命令行工具:`rosparam`，参数如下:  

- `rosparam list` 列出服务器中的所有参数。
- `rosparam get [param_name]` 获取某一参数值。
- `rosparam get /` 获取所有参数值。
- `rosparam set [param_name] [value]` 设置参数值。
- `rosparam delete [param_name]` 删除参数。
- `rosparam dump [file_name]` 将参数服务器保存到一个文件(.yaml文件)。
- `rosparam load [file_name] [namespace]` 将参数文件加载到新的命名空间。


## 开源社区级

ROS开源社区级的概念主要是关于ROS资源，能够通过独立的网络社区分享软件和知识。这些资源包括:  

- **发行版(Distribution)**: ROS发行版是可以独立安装的、带有版本号的一系列功能包集。
- **软件源(Repository)**: ROS依赖于共享开源代码和软件源的网站或主机服务，在这里，不同的机构能够发布和分享各自的机器人软件与程序。  
- **ROS Wiki**: 是用于记录有关ROS系统信息的主要论坛。
- **邮件列表(Mailing list)**: ROS用户邮件列表是ROS的主要交流渠道，能够交流从ROS软件更新到ROS软件使用中的各种疑问和信息。


# ROS的使用

## 创建工作空间

所有的ROS程序，包括我们自己开发的程序，都被组织成功能包。而ROS的功能包被存放在称之为工作空间的目录下。因此，在我们写程序之前，第一步是创建一个
工作空间以容纳我们的功能包。  

创建名为`catkin_ws`的工作空间，命令如下：  

```bash
$ mkdir -p catkin_ws/src
$ catkin_make  # 对工作空间（下的所有功能包)进行编译，这一步相当于初始化构建工作空间
```
执行完上述命令后发现`catkin_ws`目录下多出两个目录，分别是`build`和`devel`。在`devel`目录下，还可以看到很多`setup.*sh`文件。接下来输入
如下命令：  

```bash
$ source devel/setup.bash
```
这句命令的意思时把`catkin_devel`目录下的`setup.bash`文件挂载到ROS的文件系统里去，这样当用户执行一些文件系统的命令时，就不会提示找不到该
工作空间里的包或文件了。  

**注意**: 对于许多用户而言，没有必要使用多个ROS工作空间，但是，ROS的catkin编译系统会试图编译一个工作空间内的所有功能包。因此，如果你的工作
涉及大量的功能包，或者涉及几个相互独立的项目，则维护数个独立的工作空间可能是有帮助的。  

查阅资料后找到一个编译工作空间中某个包的命令，如下：  

```bash
$ cd ~/catkin_ws/
$ catkin_make -DCATKIN_WHITELIST_PACKAGES="[package_name]"
```

## 创建一个程序包

- 切换到src目录下

- 使用`catkin_create_pkg`命令创建一个名为'beginner_tutorials'的新程序包，这个程序包依赖于std_msgs、roscpp和rospy： 

```
$ catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
```

## 创建msg和srv

msg文件就是一个描述ROS中所使用消息类型的简单文本，srv文件描述一项服务，包括请求和响应两个部分


# 补充资料

## ROS中的传感器

### Imu——陀螺仪

Imu.msg 该消息为陀螺仪数据，加速度（m/s^2），角速度（rad/s），具体如下：  

- `Header header` # 时间戳与坐标系
- `geometry_msgs/Quaternion orientation` # 四元数，表示方向
- `float64[9] orientation_covariance` # 协方差
- `geometry_msgs/Vector3 angular_velocity` # 角速度
- `float angular_velocity_covariance` # 角速度协方差
- `geometry_msgs/Vector3 linear_acceleration` # 线加速度
- `float64[9] linear_acceleration_covariance` # 线加速度协方差

### Joy——游戏手柄

- `Header  header` # 时间戳
- `float32[] axes` # 轴反馈
- `int32[] buttons` # 按钮反馈

### LaserScan——激光雷达

- `Header header` # 第一束光线获取的时间戳，frame_id中，角速度增益正方向为绕z轴逆时针旋转，0角度为x轴正方向
- `float32 angle_min` # 起始角度
- `float32 angle_max` # 结束角度
- `float32 angle_increament` # 角度增益
- `float32 time_increament` # 两次测量之间的时间差，用于激光摄像头移动时，3d点的插入
- `float32 scan_time` # 两次扫描之间的时间差 
- `float32 range_min，float32 range_max` #  激光测距仪的最小距离、最大距离 （m）
- `floate32[] ranges` # 距离数组
- `float32[] intensities` # 强度数组

## geometry_msgs中的消息类型

### Point

- `float64 x`
- `float64 y`
- `float64 z`

### Point32

- `float32 x`
- `float32 y`
- `float32 z`

一般情况下使用Point，大规模点云使用Point32

### PointStamped

- `std_msgs/Header header` # 包含坐标系和时间戳信息
- `geometry_msgs/Point` # point 点

# Polygon 多边形
- `geometry_msgs/Point32[] points` 

## PolygonStamped

-` std_msgs/Header header`
- `geometry_msgs/Polygon polygon` 

## Pose

- `geometry_msgs/Point position` # 位置
- `geometry_msgs/Quaternion orientation` # 姿态，即方向

## Pose2D 2d平面的位姿

- `float64 x`
- `float64 y`
- `float64 theta`

## PoseArray 位姿序列

- `std_msgs/Header header`
- `geometry_msgs/Pose[] poses`

## PoseStamped 位姿

- `std_msgs/Header header`
- `geometry_msgs/Pose pose`

## PoseWithCovarianceStamped

- `std_msgs/Header header`
- `geomerty_msgs/PoseWithCovariance pose`

## Quaternion 四元数旋转

- `float64 x`
- `float64 y`
- `float64 z`
- `float64 w`

## Transform 坐标系之间的变换

- `geometry_msgs/Vector3 transition` # 平移向量
- `geometry_msgs/Quaternion rotation` # 旋转向量

## Twist 速度

- `geometry_msgs/Vector3 linear` # 线速度
- `geometry_msgs/Vector3 angular` # 角速度

## Wrench 力 

- `geometry_msgs/Vector3 force` # 力
- `geometry_msgs/Vector3 torque` # 扭矩

# nav_msgs中的消息类型

##　GridCells 栅格单元

- `std_msgs/Header header` # 头，时间戳与坐标系
- `float32 cell_width` # 宽度
- `float32 cell_height` # 高度

## MapMetaData 占有率栅格地图数据

- `time map_load_time ` # 地图被加载的时间
- `float32 solution` # 分辨率（m/cell）
- `uint32 width, uint32 height` # 宽度和高度
- `geometry_msgs/pose origin` # 真实世界中的原点（单位：m,m,rad），图像中的(0,0)点

## OccupancyGrid 2d栅格地图

- `Header header`
- `nav_msgs/MapMetaData info` # 地图信息
- `int8[] data` # 占有率地图数据序列，概率为【0-100】，未知为-1 

## Odometry 估计的位姿与速度

- `std_msgs/Header header` # 位姿所在的坐标系
- `string child_frame_id` # 速度所在的坐标系
- `geometry_msgs/PoseWithCovariance pose` # 位姿
- `geometry_msgs/TwistWithCovariance twist` # 速度 

## path 路径

- `Header header`
- `geometry_msgs/PoseStamped[] poses` # 代表路径的三维点坐标数组 















