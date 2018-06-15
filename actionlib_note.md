# Actionlib学习笔记

## 概述

虽然ROS服务可以不通过主题来实现和节点之间的直接通讯（向节点发送请求并接收回应），但在某些情况下，服务需要执行很长时间，而且用户希望在执行过程中取消请求，或得到关于请求进展情况的定期反馈。此时基本的服务已经不能满足用户的需求了。

Actionlib包则提供了创建服务器的工具，这些服务器执行可被抢占的长时间的目标。此外，它还提供了一个客户端接口，以便向服务器发送请求。

## 客户端与服务器的交互

ActionClient和ActionServer通过“ROS行为协议（ROS Action Protocol）”进行通信，该协议是建立在ROS消息上的。同时，客户端和服务器还提供了一个简单的API，用于在客户端请求目标或在服务器端通过函数调用和回调执行目标。

## 行为规范：Goal，Feedback和Result

为便于客户端和服务器进行通信，需要定义通信消息的规范，如下：

- Goal：为使用行为完成任务，引入了目标的概念。它可以通过一个ActionClient发送到一个ActionServer。例如，为了移动机器人，可以设置目标为一个PoseStamped类型的消息，它包括机器人的目标位置；而为了控制激光雷达，目标会是激光雷达的参数（最大角度、最小角度、扫描速度等）。

- Feedback：服务器可以使用反馈来告诉客户端目标进行的情况。在移动机器人时，反馈可以是机器人在路径上的姿态；控制激光雷达时，服务可以是扫描完成之前的时间。

- Result：完成目标后，服务器会将结果发送客户端，与反馈不同的是，结果只发送一次。当action的目的是提供某种信息时，这是非常重要的，例如激光雷达产生的点云信息。

## .action文件

行文规范使用.action文件，包括目标定义、结果定义和反馈定义，两两之间用`---`分开。

这些文件位于package的action目录下。下面是一个例子，`DoDishes.action`:
```
    # 定义目标
    uint32 dishwasher_id  # 指明要用的washer
    ---
    # 定义结果
    uint32 total_dishes_cleaned  # 共洗的盘子数量
    ---
    # 定义反馈
    float32 percent_complete  # 完成率
```
基于这个.action文件，需要生成6个message用于服务器和客户端之间的通信，这些消息会在编译过程中自动生成。

### 创建工程

- 在工作空间的src目录下创建一个新的包：

```
    catkin_create_pkg std_msgs rospy roscpp actionlib actionlib_msgs message_generation
```

- 新建action目录，在下面创建DoDishes.action文件，内容如上。

- 对于上面的文件，会由genaction.py生成如下message：
    - DoDishesAction.msg
    - DoDishesActionGoal.msg
    - DoDishesActionResult.msg
    - DoDishesActionFeedback.msg
    - DoDishesGoal.msg
    - DoDishesResult.msg
    - DoDishesFeedback.msg

这些message会被用于action服务器和客户端之间内部通信。

**注**：生成上述文件有两种方法：  

- 一种是在配置文件中添加依赖  
    - 在Cmake.list文件中设置:
```
    find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        message_generation
        actionlib_msgs
        actionlib
    )

    add_action_files(
        DIRECTORY
        action
        FILES
        DoDishes.action
    )


    generate_messages(
        DEPENDENCIES
        actionlib_msgs
    )

```

    - 在Package.xml文件中设置：  
```
  <build_depend>actionlib_msgs</build_depend>
  <build_depend>message_generation</build_depend>
  <build_export_depend>message_generation</build_export_depend> 
  <exec_depend>actionlib_msgs</exec_depend>
  <exec_depend>message_generation</exec_depend>
```
- 一种是用genaction.py文件手动生成：
```
$ roscd learn_actionlib  
$ rosrun actionlib_msgs genaction.py -o msg/ action/DoDishes.action
```

### 编写简单的Client和Server

（略,见官方的教程 [http://wiki.ros.org/actionlib/Tutorials](http://wiki.ros.org/actionlib/Tutorials)）

### actionlib_msgs

- GoalID
```
    time stamp  # 目标被请求的时间，用于服务器抢占目标？
    string id  # 提供了连接feedback和result的方法，id是唯一的
```

- GoalStatus
```
    GoalID goal_id
    uint8 status
    uint8 PENDING         = 0   # 目标已经被服务器处理了
    uint8 ACTIVE          = 1   # 目标正在被处理
    uint8 PREEMPTED       = 2   # 目标在开始并完成执行后，收到一个取消请求
    uint8 SUCCEEDED       = 3   # 服务器成功完成了目标
    uint8 ABORTED         = 4   # 由于一些原因，目标在执行过程中被放弃
    uint8 REJECTED        = 5   # 由于目标不可实现或无效，被服务器拒绝执行
    uint8 PREEMPTING      = 6   # 目标开始执行但还未完成时，收到了取消请求
    uint8 RECALLING       = 7   # 目标开始执行前被取消，但服务器还未确认其已被取消
    uint8 RECALLED        = 8   # 目标开始执行前被成功取消
    uint8 LOST            = 9   # 客户端确认目标丢失

    string text  # 允许用户将字符串与GoalStatus关联以进行调试
```

- GoalStatusArray
```
    Header header
    GoalStatus[] status_list  # 当前追踪的目标的状态序列

```
