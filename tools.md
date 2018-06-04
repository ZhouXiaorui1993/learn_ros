# 一些常用的工具

## rqt_graph

用途: 能够创建一个显示当前系统运行情况的动态图形。  

启动: `rosrun rqt_graph rqt_graph`  

## rqt_plot

用途: 可以实时显示一个发布到某个话题上的数据变化图形(类似波形图，根据消息中提供的时间戳来绘制的)。  

启动: `rosrun rqt_plot rqt_plot`  

## rqt_console

用途: 属于ROS日志框架(logging framework)的一部分，用来显示节点的输出信息，一般结合rqt_logger_level一起使用。  

启动: `rosrun rqt_console rqt_console`

## rqt_logger_level

用途: 用来修改节点运行时输出信息的日志等级(logger levels),包括(debug、info、warn和error)。  

启动: `rosrun rqt_logger_level rqt_logger_level`

## roswtf

用途： 检测给定功能包中所有组件的潜在问题。

使用： 使用`roscd`切换到想要分析的功能包目录下，然后运行`roswtf`，就可以获得检测结果。

## image_view

用途： 查看一张图像或显示在某个主题中发布的图像，可以将当前帧图像保存在硬盘里，通常会保存在home目录下或者~/.ros目录下。

使用： `rosrun image_view image_view [image:=/package_name]`

## rviz

一款3D可视化工具，能够呈现点云（PointCloud）、激光扫描（LaserScan）等3D数据。这些是很特殊的数据格式，数据相关于特定的传感器坐标空间，并通过
坐标来呈现数据。

3D数据通常使用点云格式（组织好的或未组织的），rviz工具集成了能够完成3D数据处理的OpenGL接口，能够将传感器数据在模型化世界（world）中展示。
因此，我们将会看到在复杂系统中至关重要的传感器坐标系建立和坐标变换。

启动： `rosrun rviz rviz`

### 简介

右边的Display面板是用来配置参数的，如果已有配置文件，可以通过点击File|OpenConfig来加载配置（.cvg文件）。

在Display区域下，有一个Add按钮，允许添加更多的参数项。

同时，注意到这里还有一些全局选项，基本上是用于设定固定坐标系的工具，因为坐标系是可移动的。

其次，还有轴（Axes）和网格（Grid）作为各个参数项的参照物。

同时，常用的还有标记（Marker）和点云（PointCloud2）。

在状态栏上，有时间相关的信息提示。

在上方菜单栏，有用于在虚拟环境中导航的2D Nav Goal和2D Pose Estimate。

### 主题与坐标系之间的关系

如果主题发布的数据在真实的物理世界里有一个物理位置的话，那么这样的主题就需要有一个坐标系。例如，相对于机器人质心有一段距离的某个位置上有一个加速
度计，如果我们要集成加速度数据来估计机器人的速度和位姿，那么必须对机器人质心和加速度计所在位置的坐标系进行坐标变换。在ROS系统中，带有报文头的消息
除了具有时间戳之外，还要附上frame_id（坐标系标签）。坐标系标签用于区分消息所属的坐标系。