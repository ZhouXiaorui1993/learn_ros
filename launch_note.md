# 关于launch文件的编写

## 运行

在ROS应用中，每个节点通常有许多参数需要设置，为了方便高效的操作多个节点，可以编写launch文件，然后用roslaunch命令运行。
```
roslaunch [options] [package] <filename> [arg_name:=value...]
roslaunch [options] <filename> [<filename>...] [arg_name:=value...]
```

## 格式和语法

launch文件的一般格式，参数：

```xml
<launch>
            <!-- 注释这样写 -->
            <node pkg="..." type="..." name="..." respawn="true/false" ns="..." args="..."/>
            <rosparam .../>  <!-- 操作yaml文件参数 -->
            <include file="$(find <pkg>/.../xxx.launch)"/>  <!-- 启动其它的launch文件，也可以在其中 加上<arg/>标签设定该文件的参数值-->
            <env .../>  <!-- 让该节点读入环境变量 -->
            <remap from="..." to="..."/>
            <arg name="..." value="..."/>  <!-- 其中name表示参数的名称，value是参数的值，有时也用default="..."来设定预设值 -->
</launch>
```

参数说明：

- <node> 呼叫节点，要启动的node参数

        pkg="mypackage"  #　表示节点所在的功能包
        type="nodetype"  # 表示这个节点实际的名称，也就是开发时取的名字
        name="nodename"  # 虽然也是该节点的名称，不过可以再另外帮这个节点取个新的名称，
        args="arg1 arg2 ..."  # (可选) 
        respawn="true"  #  (可选)，如果设为true，则表示当节点由于不明原因停止运行时，会自动重启节点  
        ns="foo"  # (可选)，表示在foo命名空间启动节点 
        output="log|screen" # (可选)，表示输出到日志文件或屏幕  它会将节点原名覆盖掉，以这个名称表示。可以在执行时，用rqt或rosnode list, rosnode info等指令查看到。  
        也可以在<node>中写<param>和<rosparam>，表示该节点的内部参数。

- <rosparam> 操作yaml文件参数
        
        command="load|dump|delete"  # 默认为load  
        file="$(find pkg-name)/path/foo.yaml"  # （load或dump命令）yaml文件的名字
        param="param-name"  # 参数名

- <param> 定义一个设置在参数服务器的参数，它可以添加到<node>中

        name="namespace/name"
        value="value"  # (可选)，如果省略这个参数，则指定一个文件(binfile/textfile)或命令
        type="str|int|double|boot"  # (可选)，指定参数的类型
        textfile="$(find pkg-name)/path/file" (可选)

- <include> 调用执行其他launch文件

        file="$(find pkg-name)/path/launch_file.launch"

- <env> 设置节点的环境变量

        name="environment-variable-name"
        value="environment-variable-value"
        
- <remap>  将参数名映射为另一个名字

        from="original-name"
        to="new-name"

- <arg> 定义一个局部参数，该参数只能在一个launch文件中使用

        <arg name="foo"/>   表示声明一个参数foo，后面需要给它赋值。赋值方式是运行launch文件时，在后面添加该变量和值，如`roslaunch pkg_name launch_file.name foo:=2`。
        <arg name="foo" default="1"/>  表示声明一个参数foo，如不赋值则取默认值1
        <arg name="foo" value="bar"/>  声明一个常量foo, 它的值不可修改

- 逻辑判断式 if&unless

在launch文件中也可以使用逻辑判断，但是必须搭配标签使用。如下：  

```
<arg name="load_driver" default="true"/>
<group if="$(arg load_driver)">
<include file="$(find pkg-name)/launch/xxx.launch"/>
</group>
```

同样的，可以把<group>标签中的if换成unless，该语句就变成了，直到该变量变为True时，才执行该节点或launch文档。

此时在终端执行这个launch文档时，如果要关闭执行<include>标签中的launch文档，只需输入：

```
$ roslaunch pkg-name launch_file.launch load_driver:=false
```
