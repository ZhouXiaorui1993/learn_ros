# TF学习笔记

## tf简介

### 示例程序

首先运行一个例子：

```
    $ roslaunch turtle_tf turtle_tf_demo.launch
```		

会看到出现两只小海龟，后出现的那只（turtle2）马上跟上了前面那只（turtle1），如果用键盘操纵turtle1，则turtle2会移动跟随其运动。

### 解释

这个例子利用了tf库创建了三个坐标系：世界坐标系、turtle1坐标系和turtle2坐标系。教程示例使用了一个tf broadcaster来发布了两个turtle的坐标系，一个tf listener用来计算两者之间的差异并驱动turtle2跟随turtle1。

### tf tools

#### view_frames

view_frames工具可以创建一个框图来展示tf发布的坐标系，运行它：

```
    $ rosrun tf view_frames
```

会看到：

```
    Listening to /tf for 5.000000 seconds
    Done Listening
    dot - graphviz version 2.38.0 (20140413.2041)

    Detected dot version 2.38
    frames.pdf generated
```

可见有一个tf listener正在工作，且绘制了一个表示各个坐标系连接关系的树在frames.pdf文件(保存在当前目录)中，查看它：

```
    $ evince frames.pdf  // 这是调用了系统的pdf阅读器，也可以自行打开
```
可以看到，有三个坐标系：world、turtle1和turtle2，其中，world坐标系是turtle1和turtle2的parent坐标系。

#### rqt_tf_tree

rqt_tf_tree是一个实时显示tf树的工具，可以点击refresh按钮实时刷新tf树的显示，运行它：

```
    $ rosrun rqt_tf_tree rqt_tf_tree
```

#### tf_echo

可以利用tf_echo工具来查看两个坐标系之间的变换，命令格式为：

```
    $ rosrun tf tf_echo [reference_frame] [target_frame]
```
用矩阵表示为：

T_turtle1-turtle2 = T_turtle1-world * T_turtle2-world

看一下例子里的变换：

```
    $ rosrun tf tf_echo turtle1 turtle2
```

移动海龟可以看到：

```
At time 1528082259.348
- Translation: [0.000, 0.000, 0.000]  # 平移变换
- Rotation: in Quaternion [0.000, 0.000, -0.113, 0.994]  # 旋转变换，四元数表示
            in RPY (radian) [0.000, 0.000, -0.227]  # 弧度表示
            in RPY (degree) [0.000, 0.000, -12.994]  # 角度表示
At time 1528082260.340
- Translation: [-1.389, -0.006, 0.000]
- Rotation: in Quaternion [0.000, 0.000, 0.000, 1.000]
            in RPY (radian) [0.000, -0.000, 0.001]
            in RPY (degree) [0.000, -0.000, 0.052]
At time 1528082261.349
- Translation: [-1.020, -0.271, 0.000]
- Rotation: in Quaternion [0.000, 0.000, 0.129, 0.992]
            in RPY (radian) [0.000, -0.000, 0.259]
            in RPY (degree) [0.000, -0.000, 14.862]
```

#### rviz和tf

可以利用rviz可视化工具，添加tf显示，设置parent为world来直观的查看tf变换：
```
    $ rosrun rviz rviz -d `rospack find turtle_tf`/rviz/turtle_rviz.rviz
```

此时如果移动海龟，则可以看到坐标系移动，另一个跟随移动。




























