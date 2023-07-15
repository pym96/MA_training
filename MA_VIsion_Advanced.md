# ROS_CONTROL 文档

## 构建工业级移动机械臂

假设构建机械臂底座的移动约束为两个自由度，即x轴方向的平移以及沿z轴旋转。
$$
x` = cos(wσt） - sin(wσt)
麦轮运动学模型，待补充（主要是数学符号难打）
$$
在实际开发中，将机器人的底盘视为黑匣子；一般需要至少两个话题来记录机器人的运动，他们的作用可以简单理解为机器人”怎么样到哪去“以及机器人”实际走到了哪“，第一个可以由话题”/cmd_key"接受，用来控制机器人的运动，机器人运动后将自身的运动信息再发布给里程计话题"/odom"流程如下：

```
"/cmd_key"->【移动机器人底座】 -> “/odom" 
```

### 1."/cmd_key" 消息类型： geometry_msgs  / Twist

```
`geometry_msgs/Twist`是ROS（Robot Operating System）中定义的消息类型之一，用于表示机器人或运动对象的线性和角度速度。

`Twist`消息类型包含以下字段：
- `Vector3 linear`: 表示线性速度的三维向量，具有 `x`、`y` 和 `z` 分量。`x` 分量表示沿着机器人的前进方向的线性速度，`y` 分量表示沿着机器人的左侧方向的线性速度，`z` 分量表示沿着机器人的上下方向的线性速度。
- `Vector3 angular`: 表示角度速度的三维向量，具有 `x`、`y` 和 `z` 分量。`x` 分量表示绕机器人的前进方向旋转的角度速度，`y` 分量表示绕机器人的左侧方向旋转的角度速度，`z` 分量表示绕机器人的上下方向旋转的角度速度。

通过使用 `geometry_msgs/Twist` 消息类型，可以发布机器人的速度命令，例如指定机器人沿着特定方向移动或旋转的速度。
```

### 2."/odom" 消息类型： nav_msgs/Odometry

```
`nav_msgs/Odometry`是ROS中定义的消息类型之一，用于表示机器人的里程计信息，即机器人在运动过程中的位置和姿态。

`Odometry`消息类型包含以下字段：
- `std_msgs/Header header`: 包含时间戳和坐标系信息等标准消息头。
- `string child_frame_id`: 子坐标系的名称，表示相对于父坐标系的坐标系，通常用于表示机器人或传感器的框架。
- `geometry_msgs/PoseWithCovariance pose`: 机器人的位姿信息，包括位置和姿态，以及位置和姿态的协方差矩阵。
- `geometry_msgs/TwistWithCovariance twist`: 机器人的速度信息，包括线性速度和角度速度，以及线性速度和角度速度的协方差矩阵。

通过使用`nav_msgs/Odometry`消息类型，可以发布机器人的里程计信息，例如机器人的当前位置、姿态以及速度信息。该消息类型在导航和定位系统中被广泛使用，用于估计机器人的运动状态和进行路径规划等任务。
```

### 3.svg图片

```c++
// 在实际开发中，我们会在很多源码中的某一个 /doc文件夹下看到.svg图片，它的作用到底什么呢？、

在ROS 2（机器人操作系统2）的结构中，.svg文件通常代表可缩放矢量图形文件（Scalable Vector Graphics）。SVG是一种基于XML的矢量图形格式，用于描述二维图形和图形应用程序。虽然SVG文件本身不是ROS 2特有的，但它可以在ROS 2生态系统中用于各种目的。

在ROS 2的背景下，.svg文件可以用于可视化图形用户界面（GUI），例如用户界面中不同组件的布局和外观。ROS 2提供了用于构建GUI的工具和库，.svg文件可以作为这些界面中的图形资源使用。

例如，如果您正在开发一个包含自定义用户界面的ROS 2应用程序，您可以使用.svg文件来定义在用户界面中显示的图标、按钮或其他视觉元素。.svg文件可以存储矢量图形数据，这些数据可以在用户界面中按比例缩放和渲染。

需要注意的是，在ROS 2中，.svg文件的具体用途和功能可能会根据上下文、具体的应用程序或使用的框架而有所不同。因此，.svg文件的确切目的将取决于正在开发的ROS 2项目以及它如何与应用程序的图形组件集成。
```

### 

# 知识补充：

## 差分运动学

```
差分运动学（Differential Kinematics）是机器人运动学中的一个分支，用于描述和分析差分驱动机器人（Differential Drive Robot）的运动行为。差分驱动机器人是一种具有两个并列轮子的机器人，通常一个轮子位于机器人的左侧，另一个轮子位于机器人的右侧。

差分运动学关注机器人的轮子运动和机器人的整体运动之间的关系。它通过分析和计算机器人轮子的线速度（linear velocity）和角速度（angular velocity）之间的关系，来确定机器人的整体运动。

具体来说，差分运动学使用以下几个基本概念和关系：
- 左右轮子的线速度：分别表示为v_left和v_right，表示左轮和右轮的线速度。
- 机器人的线速度和角速度：表示为v（机器人的线速度）和ω（机器人的角速度）。
- 轮子半径：表示为r，表示机器人轮子的半径。
- 轮距：表示为L，表示机器人两个轮子之间的距离。

根据差分运动学的原理和几何关系，可以推导出以下关系：
- 机器人的线速度v可以通过左右轮子的线速度v_left和v_right来计算：v = (v_left + v_right) / 2。
- 机器人的角速度ω可以通过左右轮子的线速度v_left和v_right以及轮距L来计算：ω = (v_right - v_left) / L。

差分运动学使我们能够通过控制左右轮子的线速度来实现机器人的整体运动，例如直线运动、旋转运动和曲线运动等。它为差分驱动机器人的控制和路径规划提供了重要的基础。
```

# ROS2知识

## CMAKE in ros2

**一般初学者会对ros2的CMake感到一头雾水**，建议学习以下两个·教程

1. 想从头开始的可以看这篇文章: 如何在ROS2中手写、编译、链接自己的库

   ```
   https://zhuanlan.zhihu.com/p/437550838
   ```

   

   2.想方便又快速的可以看这篇文章 ament_cmake_auto	

```
https://zhuanlan.zhihu.com/p/438191834
```



# ROS 1 知识

## 辅助功能包的介绍

### 1. rospack 

```c++
// 用来寻找包的信息

rospack find [package_name]

// 比如 rospack find roscpp，在 4号NUC的输出结果就是：/opt/ros/noetic/share/roscpp
```

### 2.roscd

```c++
// 可以把它理解为“超级” cd 命令吧，但仅限于ros, 比如 roscd roscpp 就可以直接跳到相关ros版本的share目录下
```

## catkin package

```c++
// catkin 的作用和 ros2 的colcon 很像我觉得， 以后谁有不同理解欢迎来打我的脸（From Pan)
```

### 1.catkin_create_pkg2.catkin_make

```c++
// 这时要回到 /src的上级目录， 比如catkin_create_pkg 时我们在 ~/ros_ws/src下，这时候要换到 ~/ros_ws下
```

### 3.  ~./ros_ws/devel/setup.bash

```c++
// 就像 ros2 colcon build之后要 . install/setup.bash一样， ros1 在 catkin_make 之后也有：source , . ~/catkin_devel_setup.bash
```

```c++
// 在 rospack depends <package_name> 之后，你会找到所依赖的包
```

## 如果看官方文档看不下去也可以看看下边这个

```
https://zhuanlan.zhihu.com/p/384578650
```

# Nav2

## Intro: Nav2 is the professionally supported spiritual successor of the ros navigation stack

## 1. 以后的学习难度将会珠穆朗玛锋式上升，所以先来点小学生都会的

```c++
// 1. 确保安装好了ros2
https://navigation.ros.org/
```

# Ros URDF



```c++
// 在学习URDF之前，请先了解tf2, ros 的所有位姿都是parent frame 和 child frame 之间的变化
```

## UDRF xml 文件的基本结构

```xml
<?xml version="1.0"?>
<robot name="Your robot name">
	<link name="demo">
        
        <visual>
     这是我们在 RViz 和 Gazebo 中看到的内容。 我们可以明确三个方面：

     几何体 - 具有尺寸参数的盒子/圆柱体/球体，或网格
     原点 - 几何体的偏移量，因此不需要以链接原点为中心
     材质 - 基本上是颜色。 我们可以指定声明材料的名称，或者直接描述颜色。 （请注意，这将在 RViz 中设置颜色，但不会在 Gazebo 中设置颜色)
        </visual>
        
        
        <collision>
     用于物理碰撞计算。 我们可以设置：

     几何和原点 - 与visual选项相同。 这通常是从视觉标签复制粘贴的，但是出于计算原因，我们可能需要更简单的碰撞几何体（例如，盒子而不是网格）。
        </collision>
        
        
        <inertial>
     这也用于物理计算，但确定链接如何响应力。 惯性特性为：

     质量 - 链接的质量
     原点 - 质心（又名重心）。 这是链接可以“平衡”的点（3D 中的一个稍微容易混淆的概念）。 对于大多数简单的情况，这只是中心（与visual/collision相同的原点）。
     惯量 - 转动惯量矩阵。 这可能是本节中最令人困惑的部分。 它描述了质量分布如何影响旋转。 此处提供了常见形状的矩阵列表。 通过将我们的链接近似为棱柱、椭球体或圆柱体，可以实现相当准确的模拟，但是对于大多数目的，非常粗略的猜测就可以了。 另外，不要将惯性和惯性混淆 - 惯性矩阵只是惯性属性的一部分。
        </inertial>
        
        <visual>
        	<geometry>
            <origin>
            <material
        </visual>
                
        <collision>
        	<geometry>
            <origin>
        </collision>       
        
        <inertial>
            <mass>    
        	<origin>
          	<inertia> 
        </inertial>
    </link>

                
   # 注意：joint的type是根据tf2 中parent frame 和 child frame之间的关系来定义的
   # 1. revolute(rolled backward or downward)
   # 2. continuous: parent frame 不动，child frame 一直在动， 类比车轮
   # 3. prismatic: child frame 相较于 parent frame 有一个 linear movement, 比如 linear actuator, 如果还不理解就去看看飞镖的电机运动
   # 4. fixed: child frame 相较于 parent frame 并不移动          
                
                
   # Revolute - A rotational motion, with minimum/maximum angle limits.
   # Continuous - A rotational motion with no limit (e.g. a wheel).
   # Prismatic - A linear sliding motion, with minimum/maximum position limits.
   # Fixed - The child link is rigidly connected to the parent link. This is what we use for those “convenience” links.

    <joint name="demo_joint" type="revolute">
        <parent link="silder_link"/>
        <child link="arm_link"/>
        <origin xyz=d"0.25 0 0.15" rpy="0 0 0"/> # rpy: roll pitch yaw
        <axis xyz="0 -1 -"/>
        <limit lower="0" upper="${pi/2}" velocity="100" effort="100"/>
     </joint>

</robot>
            
            
   <joint>
    	<limit>
       	Limits - Physical actuation limits, which may be expected by other parts of the system. These can include:

    	Upper and Lower position limits - in metres/radians
    	Velocity limits - in m/s or rad/s
    	Effort limits - in N or Nm
       </limit>        
            
    </joint>
```

## Using xacro to write URDF eaiser

```xml
# All things are stored in the name.urdf.xacro file


在我们查看 URDF 的示例之前，有必要了解一下 ROS 提供的一个工具，该工具使它们更易于编写，称为 xacro（XML 宏的缩写）。 Xacro 允许我们以各种方式操作 URDF 文件，但我们将看到的两个主要方法是将代码拆分为多个文件，并避免重复代码。

为了在我们的文件中使用 xacro，我们需要做的就是确保我们的 robots 标签包含一个额外的位，如下所示。 （您可能会注意到这个特定的机器人标签没有名称 - 下面将详细介绍！）最简单的方法是将其放入每个机器人标签中，这样您就可以随时选择使用 xacro。


<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
    
    
# 为什么要用xacro？
	能够将 URDF 拆分为多个文件非常有用，因为它们可能会变得相当大且难以处理。 将不同的组件分开可以更轻松地查找和更改内容，并且在使用版本控制软件（如 Git）时可以快速了解更改的内容。

我们如何拆分 URDF 将取决于各个开发人员及其硬件，但不同文件的一个示例可能是：

     核心机器人（连杆和关节）
     用于展示的材料（颜色）列表
     传感器链接/关节及其相应的 Gazebo 标签
     宏（下一节将详细介绍）
    
# 为了在自定义使用xacro 文件，我们需要在robot 标签中包含一个特定的机器人标签：
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
    
# 然后，每当我们想要实际使用 URDF 时，我们需要首先对这些文件运行 xacro 软件，该软件会将它们处理成一个完整的 URDF“文件”（临时在内存中）。 我们在上一个教程中运行 robots_state_publisher 时看到了这一点。 即使我们有多个文件进入，它也会作为一条消息发布到 robots_description 主题。 通常我们使用像这样的launch文件来为我们完成这一切。(例子如下)
 
    
    
```

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'urdf_example'
    file_subpath = 'description/example_robot.urdf.xacro'


    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()


    # Configure the node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}] # add other parameters here if required
    )


    # Run the node
    return LaunchDescription([
        node_robot_state_publisher
    ])
```



## Extra tags

```xml
机器人、链接和关节标签是构成 URDF 文件的主要标签，但我们不仅限于这些！ 还有一些其他标签是 URDF 规范的一部分，但我们也可以添加我们喜欢的任何其他标签，如果不需要，它们将被忽略。 某些节点会期望这些额外的标签并可以使用它们。

您可能会遇到的一些常见额外标签是：


<material>
	 - Allows us to give a “name” to a colour once, and then reuse that name in as many links as we like
</material>

<gazebo> 
      - Lets us specify certain parameters that are used in the Gazebo simulation environment 
</gazebo>


<transmission> 
	- Provides more detail about how the joints are driven by physical actuators
</transmission>
```

# 深度学习 + 自瞄

## 什么是 openvino?

```
OpenVINO（Open Visual Inference and Neural Network Optimization）是英特尔开发的一种开源工具套件，用于优化和加速深度学习模型的推理。它旨在提高深度神经网络在各种硬件平台上的性能，并支持嵌入式设备、物联网设备和边缘计算设备等多种应用场景。

OpenVINO提供了一套用于模型优化和推理的工具和库，以最大限度地利用英特尔的硬件加速器，如CPU、GPU、FPGA和VPU（视觉处理器单元）。它支持多种深度学习框架，如TensorFlow、Caffe、ONNX（开放神经网络交换）等，并提供了针对英特尔硬件的优化和加速功能。

使用OpenVINO，开发人员可以将训练好的深度学习模型转换为针对特定硬件平台的可优化格式，并使用英特尔硬件加速器执行推理。这种优化和加速可以显著提高推理性能，从而实现实时或近实时的深度学习应用。

总结起来，OpenVINO是一种用于优化和加速深度学习模型推理的开源工具套件，它提供了针对英特尔硬件的优化功能，以提高深度学习应用在各种设备上的性能表现。
```

**以下是openvino的文档**

```
https://docs.openvino.ai/2022.3/get_started.html
```

## Exporting a PyTorch Model to ONNX Format

```
https://docs.openvino.ai/2022.3/openvino_docs_MO_DG_prepare_model_convert_model_Convert_Model_From_PyTorch.html
```
