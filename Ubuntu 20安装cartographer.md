```
sudo gedit /etc/apt/sources.list
```

对于官网的源，不可以删掉，在下面添加这些源:

# 阿里源
```
deb http://mirrors.aliyun.com/ubuntu/ bionic main restricted universe multiverse
deb http://mirrors.aliyun.com/ubuntu/ bionic-security main restricted universe multiverse
deb http://mirrors.aliyun.com/ubuntu/ bionic-updates main restricted universe multiverse
deb http://mirrors.aliyun.com/ubuntu/ bionic-proposed main restricted universe multiverse
deb http://mirrors.aliyun.com/ubuntu/ bionic-backports main restricted universe multiverse
deb-src http://mirrors.aliyun.com/ubuntu/ bionic main restricted universe multiverse
deb-src http://mirrors.aliyun.com/ubuntu/ bionic-security main restricted universe multiverse
deb-src http://mirrors.aliyun.com/ubuntu/ bionic-updates main restricted universe multiverse
deb-src http://mirrors.aliyun.com/ubuntu/ bionic-proposed main restricted universe multiverse
deb-src http://mirrors.aliyun.com/ubuntu/ bionic-backports main restricted universe multiverse
#清华源
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic-updates main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic-backports main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic-security main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic-proposed main restricted universe multiverse
deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic main restricted universe multiverse
deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic-updates main restricted universe multiverse
deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic-backports main restricted universe multiverse
deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic-security main restricted universe multiverse
deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic-proposed main restricted universe multiverse
#中科大源
deb https://mirrors.ustc.edu.cn/ubuntu/ bionic main restricted universe multiverse
deb https://mirrors.ustc.edu.cn/ubuntu/ bionic-updates main restricted universe multiverse
deb https://mirrors.ustc.edu.cn/ubuntu/ bionic-backports main restricted universe multiverse
deb https://mirrors.ustc.edu.cn/ubuntu/ bionic-security main restricted universe multiverse
deb https://mirrors.ustc.edu.cn/ubuntu/ bionic-proposed main restricted universe multiverse
deb-src https://mirrors.ustc.edu.cn/ubuntu/ bionic main restricted universe multiverse
deb-src https://mirrors.ustc.edu.cn/ubuntu/ bionic-updates main restricted universe multiverse
deb-src https://mirrors.ustc.edu.cn/ubuntu/ bionic-backports main restricted universe multiverse
deb-src https://mirrors.ustc.edu.cn/ubuntu/ bionic-security main restricted universe multiverse
deb-src https://mirrors.ustc.edu.cn/ubuntu/ bionic-proposed main restricted universe multiverse
```

添加完后，保存并关闭该文档。

在终端输入：

```
sudo apt update
sudo apt upgrade
```

1.先安装一些工具：wstool, rosdep和Ninja:
在终端输入：

```
sudo apt-get update
sudo apt-get install -y python3-wstool python3-rosdep ninja-build stow
```


2. 加入谷歌服务器域名
由于从google直接下载代码包可能会访问失败，所以需要做如下修改：

```
sudo gedit /etc/resolv.conf
```

将原有的nameserver这一行注释，并添加以下两行：

```
nameserver 8.8.8.8 
nameserver 8.8.4.4 
```

然后保存并关闭文件。（更改后，浏览器可能无法正常访问网页）

3.下载三个代码包
 创建一个cartographer_ros  的工作空间，名字叫作 ‘catkin_ws‘：

```
mkdir catkin_ws
cd catkin_ws
```

```
wstool init src
```

由于我按照官方教程在使用wstool工具下载安装包时，使用命令

wstool merge -t src https://raw.githubusercontent.com/cartographer-project/cartographer_ros/master/cartographer_ros.rosinstall
会报错（访问被拒绝）。于是选择分别从git地址clone三个代码包：

```
  cd src
  git clone https://github.com/googlecartographer/cartographer_ros.git
  git clone https://github.com/googlecartographer/cartographer.git
  git clone https://github.com/ceres-solver/ceres-solver.git
```

使用以上方法可以下载成功

在src文件夹下面可以看到三个代码包



4.安装依赖项：

```
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
```

执行完可能会报错，但是这个错误可以忽略不管。

安装abseil-cpp 库

继续输入以下命令：

```
src/cartographer/scripts/install_abseil.sh
sudo apt-get remove ros-${ROS_DISTRO}-abseil-cpp
```

5.编译链接

```
catkin_make_isolated --install --use-ninja
```

链接成功后，即可下载cartographer 例子数据包进行测试

6.下载cartographer 例子数据包
2D数据包下载的命令如下：

```
wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/backpack_2d/cartographer_paper_deutsches_museum.bag
```

3D数据包下载的命令如下：

```
wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/backpack_3d/with_intensities/b3-2016-04-05-14-14-00.bag
```


7. launch数据包
直接执行launch语句来run数据包，可能会报错，所以通常加一句：

```
source install_isolated/setup.bash
```

launch 2D：

```
roslaunch cartographer_ros demo_backpack_2d.launch bag_filename:=${HOME}/Downloads/cartographer_paper_deutsches_museum.bag
```

launch 3D:

```
roslaunch cartographer_ros demo_backpack_3d.launch bag_filename:=${HOME}/Downloads/b3-2016-04-05-14-14-00.bag
```

执行成功后，3D效果如下：



8.将服务器域名恢复原状
输入命令：

```
sudo gedit /etc/resolv.conf
```

将原来注释掉的nameserver这一行，把注释去掉

将后来加入的

```
nameserver 8.8.8.8
nameserver 8.8.4.4
```

注释起来。

这样，浏览器又可以正常访问网页了。
