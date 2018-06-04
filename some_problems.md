# 学习中遇到的问题汇总

## Day-1

- 通过主题交互的两个节点必须位于同一个功能包吗?
- 节点名称是怎么确定的?为什么用`rosrun`运行的节点名称和用`rosnode list`得到的不一样?

## Day-2

- 昨天创建的`chapter2_tutorials` package，是按照书上的教程，基于 `rosbuild system` 创建并编译的。经过查阅资料，发现官方推荐基于 
`catkin build system` 的方式来创建和编译package，于是重新建立了 `beginner_tutorials` package。

- 在编写消息和服务之前，需要手动修改`package.xml`和`CMakeList.txt`文件，而前者涉及到了format不同影响，内容表达形式不同的问题，这里有一个
网站([Migrating from format 1 to format 2](http://docs.ros.org/indigo/api/catkin/html/howto/format2/migrating_from_format_1.html#migrating-from-format1-to-format2))
解释了两者之间的迁移问题。

- 按照官方教程进行到"编写简单的Service和Client (Python catkin)"部分时，运行`add_two_ints_server.py`后，提示:  

```
Traceback (most recent call last):
  File "/home/zhouxiaorui/catkin_ws/src/beginner_tutorials/scripts/add_two_ints_server.py", line 6, in <module>
    from beginner_tutorials.srv import *
ImportError: No module named beginner_tutorials.srv

```
经过查阅资料，得到的有效的解决方法:  

1. 是将server和client文件中的:  

```
from beginner_tutorials.srv import *
```
修改为:  

```
from beginner_tutorials.srv import *
```    

2. 是删除了`catkin_ws`中的`build`和`devel`文件夹，使用`catkin_make`重新进行了编译，打开了新的终端运行，问题解决。  

虽然问题得到了解决，但具体原因仍不太清楚。 

3. 重新安装系统后再进行到这个环节，依然出现该问题，方法二没有起作用，查阅资料得到方法三有效：即启动rosrun之前先执行以下命令：  
```bash
$ source ~/catkin_ws/devel/setup.bash
``` 
尝试将该命令加入到`~/.bashrc`中，再启动新终端运行无误，因此，应该是环境变量设置的问题，此方法应为正解。

## Day3-Day4

- 在给机器人通电以后，尝试按照说明手册下载turtlerobot包，下载失败。当前的kinetic版本似乎不能很好的支持该包，安装起来也比较麻烦，估计以后也会
不断踩坑，于是选择了重装系统为Ubuntu14.04（对应ros版本可安装indigo）。在装系统的过程中，遇到了一些问题：  

- 像上次一样在Windows下利用软碟通只做了Ubuntu的启动盘，备份了一些文件 以后开始安装。插上U盘，开机，在logo页面多次敲击F12，进入选择启动
项的页面。选择从U盘启动，进入Ubuntu安装页面。选择卸载Ubuntu16.04并重新安装后，提示将删除11个分区，并使用整个磁盘（金士顿的ssd盘，原Win
dows和Linux共用的系统盘）。看到提示后有点担心，毕竟以前也曾出过错。但搜索过后未找到合适的解决方案（感觉这时如果进入“其它选项”来重新分区的
话说不定没关系），于是继续安装，安装结束后发现整个ssd都变成了Ubuntu的系统盘，Windows系统被删除了。接下来就是漫长的重装Windows（重装过程
中删除了所有的Ubuntu分区，并留出了75G的空闲分区），装好以后在Windows的基础上重装Ubuntu。装的过程中，又遇到奇怪的提示（检测到本机上似乎
没有操作系统），查阅资料过后，说是可能是因为有多个Windows主分区的缘故，想要转换一下但没有找到合适的方法。于是硬着头皮直接选了其它选项，去
给Ubuntu分区（这里只分了三个区——4G的交换分区,200M的逻辑分区挂载到/boot，剩下的全部作为主分区挂载到/，并没有挂载/home，重装系统时备份一
下就好了，反正处处都是坑，就算挂载了也不敢不备份），然后继续安装。安装过程漫长而令人忐忑，但最后神奇的安装成功了，重启也可以找到Windows的
引导，感谢夜斗神！  

- 装好系统以后，首先选择装ros，毕竟是为了它才重装的系统。然而刚开始安装便出现了问题，按照官网的步骤执行到`sudo apt-get update`时，经常
会提醒某些包“hash 校验和不符”，执行过程中会被忽略。查了很多资料，有说换源的，有说翻墙开VPN的，还有说利用新立得软件更新器更新一下现有的软件
的，不一而足。除了VPN感觉比较麻烦没有进行尝试，其他的都试过了，问题并未得解决。后来怀疑时实验室网络的问题，换了BUAA WiFi，再进行update后
发现校验和不符的提示没有了，但仍会忽略一些包。没有办法，就在此基础上尝试进行了下面的安装步骤，网速很慢，没有等待他安装完毕便回宿舍休息了。早
上过来发现，还是有问题，有几个包总是会提示无法下载，再次尝试过后也是一样。查了一些资料，尝试自己用`wget xxx.deb`一个个将其下载了，没有问题。
然后加上`--fix-missing`参数再次进行安装，即:
```
$ sudo apt-get install ros-indigo-desktop-full --fix-missing
```
安装成功！虽然具体哪个环节起作用了不太清楚，但还是感谢夜斗神！

## Day5

- 在终端输入rosrun命令，尝试使用tab键补全时，突然报错：  
```bash
$ rosrun g[rospack] 
Warning: error while crawling /home/zhouxiaorui: boost::filesystem::status: Permission denied: "/home/zhouxiaorui/.gvfs"
```
解决方法：  
```bash
$ sudo umount /home/zhouxiaorui//.gvfs
$ rm -rf .gvfs/
```
输入以上命令，即可。

- 下载了ros书上的源码，准备按教程实验，将文件夹放到了catkin_ws工作空间的src文件夹下，但编译时遇到问题，提示缺少依赖包。百度过后，按照提示顺序
依次安装了所缺少的包，但最后一个：  
```bash
CMake Error at /opt/ros/indigo/share/catkin/cmake/catkinConfig.cmake:83 (find_package):
  Could not find a package configuration file provided by "dynamixel_msgs"
  with any of the following names:

    dynamixel_msgsConfig.cmake
    dynamixel_msgs-config.cmake
```
用`sudo apt-get install`安装时，提示无法找到该包。搜索无果，故暂且作罢。

- 上午尝试安装linux版本的网易云音乐，由于依赖问题（下载的是基于Ubuntu16.04的deb文件），未能安装成功，因此放弃了安装。但下午在安装其他包时，
系统检测到了原来的失败文件，仍然提示有问题，影响到了其他包的安装，查阅资料后，发现，使用：  
```bash
$ apt-get install -f
```
可以修复该问题，执行过后会自动删除安装失败的文件。  

## Day6

- 昨天的第二个问题解决了，原因是命令输入有误，应该是以下格式：  

```bash
$ sudo apt-get install ros-indigo-dynamixel-msgs
```
后面提示的错误也依照该格式安装。

安装过后，再次进行编译，又出现新的错误，不知如何解决。看了一下，是chapter5_tutorials包中出现的，于是干脆删除了该包，先编译通过后面用到
chapter5再说。

## Day7

- 看了一下小车的速度控制文件`turtlebot_telelop_key`，想要做几个注释，但发现该文件是只读的，为了防止以后被改错，没有选择修改文件权限，而是
切换到了root用户，对其进行了改写。

## Day8

- 控制律的编写需要解微分方程，查阅资料后知道Python的scipy包有进行科学计算的功能，但在用pip进行安装后，发现，该包被安装在了Python3下，
Python2下没有。怎么明确安装的模块对应的Python版本呢？搜索到如下解决办法：  

```bash
$ sudo python2 -m pip install xxx
$ sudo python3 -m pip install xxx  # 或利用pip3安装
```

另外一个方法是，可以通过Pycharm的package管理器来安装，入口为：File-setting-project interpreter，选择对应版本的解释器，点击面板右上的
绿色“+”号即可搜索相关的模块并下载。

## Day9 

在看导航包里的`move_base_square.py`文件时，对其中的`rospy.do_shutdown()`函数有一些疑惑，不清楚在其中注册的函数是不是每次退出程序前都会被调用，
因此想要设置断点调试一下。感觉在命令行调试有些麻烦，所以想用pycharm运行该脚本。但是在默认设置的情况下，Pycharm是找不到ros的环境变量的，所以导入
rospy会出错。

经过查阅资料，发现可以做如下设置，使得Pycharm可用于运行ros脚本：

- 编辑Pycharm的配置文件:
```
$ gedit ~/.local/share/applications/jetbrains-pycharm-ce.desktop
```
在`Exec=`后面添加`bash -i -c`， 如果原文件`Exec=`后面不是`/home/zhouxiaorui/文档/Pycharm-community/pycharm-community-2017.3.4/bin/pycharm.sh`，
则需要将其也添加进去，否则单击快捷方式无法打开Pycharm。

即最终效果为：

```
Exec=bash -i -c /home/zhouxiaorui/文档/Pycharm-community/pycharm-community-2017.3.4/bin/pycharm.sh ...(后面包括java文件等路径信息，略)
```
## Day10

**关于linux更新的问题**

像往常一样，开机后提醒系统有些内容需要升级。选择升级后提示“磁盘空间不足”：

这个更新需要花去 75.4 M 磁盘上总计 /boot 的空间。请在 19.6 M 磁盘上留出 /boot 空间。清空您的回收站和临时文件，用“sudo apt-get clean”清理以前的安装文件。

搜索后得到答案：

step1： 使用命令`dpkg --get-selections|grep linux` 查看安装的系统内核，得到的结果中带image的是系统内核。

```
zhouxiaorui@zhouxiaorui-Lenovo-IdeaPad-Y400:~$ dpkg --get-selections|grep linux-image
linux-image-3.13.0-143-generic			install
linux-image-3.13.0-144-generic			install
linux-image-3.13.0-145-generic			install
linux-image-3.13.0-32-generic			deinstall
linux-image-extra-3.13.0-143-generic		install
linux-image-extra-3.13.0-144-generic		install
linux-image-extra-3.13.0-145-generic		install
linux-image-extra-3.13.0-32-generic		deinstall
linux-image-generic				install
```
step2： 使用命令`uname -a` 查看系统当前使用的内核。

```
zhouxiaorui@zhouxiaorui-Lenovo-IdeaPad-Y400:~$ uname -a
Linux zhouxiaorui-Lenovo-IdeaPad-Y400 3.13.0-145-generic #194-Ubuntu SMP Thu Apr 5 15:20:44 UTC 2018 x86_64 x86_64 x86_64 GNU/Linux
```
可以看到当前系统使用的内核为`linux-image-3.13.0-145-generic`。

step3： 使用命令`sudo apt-get remove 安装包`删除多余的内核（因为系统自动升级会安装多个内核，卸载多余的内核可以获得更多的磁盘空间）。

```
sudo apt-get remove linux-image-3.13.0-143-generic
正在读取软件包列表... 完成
正在分析软件包的依赖关系树       
正在读取状态信息... 完成       
下列软件包是自动安装的并且现在不需要了：
  freepats gstreamer0.10-plugins-bad libcdaudio1 libfftw3-double3 libflite1
  libgme0 libgstreamer-plugins-bad0.10-0 libmimic0 libmms0 libofa0
  libopenal-data libopenal1 libqgsttools-p1 libqt5multimedia5-plugins
  libqt5multimediawidgets5 libqt5x11extras5 libslv2-9 libsoundtouch0
  libspandsp2 libvo-aacenc0 libvo-amrwbenc0 libwildmidi-config libwildmidi1
  libzbar0 linux-headers-3.13.0-143 linux-headers-3.13.0-143-generic
Use 'apt-get autoremove' to remove them.
下列软件包将被【卸载】：
  linux-image-3.13.0-143-generic linux-image-extra-3.13.0-143-generic
  linux-signed-image-3.13.0-143-generic
升级了 0 个软件包，新安装了 0 个软件包，要卸载 3 个软件包，有 72 个软件包未被升级。
解压缩后将会空出 198 MB 的空间。
您希望继续执行吗？ [Y/n]
...(省略以下内容)
```

类似的，执行`sudo apt-get remove linux-image-3.13.0-144-generic `卸载另一个无用内核。

step4： 很重要的一步，使用命令`sudo update-grub`更新开机启动项，一定要执行，否则会导致系统无法启动！

```
zhouxiaorui@zhouxiaorui-Lenovo-IdeaPad-Y400:~$ sudo update-grub
Generating grub configuration file ...
Found linux image: /boot/vmlinuz-3.13.0-145-generic
Found initrd image: /boot/initrd.img-3.13.0-145-generic
Found Windows Boot Manager on /dev/sda2@/EFI/Microsoft/Boot/bootmgfw.efi
Adding boot menu entry for EFI firmware configuration
done
```













