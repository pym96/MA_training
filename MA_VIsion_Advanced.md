# C++ 部分

## 1. attribute in the rail of struct

```c++
struct MyStruct{
  // define something here
}__attribute__((something));

/** 例如，C++ 中对 `__attribute__` 常见的使用包括：

`__attribute__((packed))`：该属性告诉编译器最小化结构体成员之间的填充，以减少内存使用。它确保编译器在结构体成员之间不插入任何填充字节。

`__attribute__((aligned(x)))`：该属性指定结构体或变量的对齐要求，确保它在内存中按照 'x' 字节的倍数对齐。

`__attribute__((noreturn))`：该属性告诉编译器一个函数不会返回给调用者。它通常用于执行像 `exit()` 或无限循环等操作的函数。

`__attribute__((deprecated))`：该属性将一个函数、变量或类型标记为已废弃，表示不建议使用它，而应使用替代选项。

`__attribute__((unused))`：该属性抑制关于未使用变量或函数的警告，表示编译器明确知道该实体被有意地不使用。

值得注意的是，`__attribute__` 的使用是特定于编译器的，其行为可能在不同版本的 GCC 或其他编译器之间有所不同。过度依赖这一特性可能使你的代码不够可移植且难以维护，因此通常建议避免使用特定于编译器的扩展，如果可能的话。

如果需要为结构体或函数提供特定属性并希望保持可移植性，考虑使用标准的 C++ 机制，比如 `alignas`、`[[nodiscard]]` 或 C++11 及以上版本中的 `[[deprecated]]` 属性，这些机制更加标准化和广泛支持。
*/
```



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

## IMU (Inertial measurement unit)

你可以粗浅的将其理解为基于牛顿力学测量单元，其具体定义如下：
IMU（惯性测量单元）是一种传感器组合，通常包括加速度计、陀螺仪和磁力计，用于测量和提供关于物体的姿态、加速度和角速度等信息。理解IMU数据可以帮助你获取和分析物体的运动和姿态信息。

下面是一些关键的概念和步骤，帮助你理解和解释IMU数据：

    坐标系：IMU数据通常以三个坐标轴表示。常见的坐标系是右手坐标系，其中X轴指向前方，Y轴指向左侧，Z轴指向上方。在使用IMU数据时，你需要了解坐标系的定义，以正确解释和使用数据。
    **视觉组的三个基本的坐标系: 世界坐标系， 云台坐标系， 相机坐标系均基于此。**
    
    加速度计数据：加速度计测量物体在三个轴向上的加速度。数据通常以米/秒^2（m/s^2）为单位表示。正值表示正方向的加速度，负值表示反方向的加速度。根据物体的运动状态和方向，你可以通过分析加速度计数据获得关于物体的加速度和动作信息。
    
    陀螺仪数据：陀螺仪测量物体绕各轴旋转的角速度。数据通常以弧度/秒（rad/s）为单位表示。正值表示顺时针旋转，负值表示逆时针旋转。通过分析陀螺仪数据，你可以获得关于物体的旋转速度和方向信息。
    
    磁力计数据：磁力计测量物体周围的磁场强度。数据通常以特定的单位（例如微特斯拉，μT）表示。通过分析磁力计数据，你可以获取关于物体相对于地球磁场的方向和姿态信息。
    
    数据融合和姿态估计：IMU数据通常需要进行数据融合和处理，以获得更准确和稳定的姿态估计。这通常涉及使用滤波器（如卡尔曼滤波器或互补滤波器）来将加速度计、陀螺仪和磁力计数据结合起来，估计物体的姿态。
    
    可视化和应用：通过将IMU数据与其他传感器数据（例如相机数据或GPS数据）结合使用，你可以实现更高级的应用，如姿态控制、导航和姿势识别等。可视化工具和算法库（如ROS中的rviz、robot_localization包）可以帮助你可视化和分析IMU数据，以及与其他传感器数据进行集成和展示。

## Quaternion

https://eater.net/quaternions/video/intro
https://www.youtube.com/watch?v=zjMuIxRvygQ



# ROS General knowledge （ROS 通用知识)

## 通信

### Service 
```c++
Service 是一种同步通信机制，用于节点之间的请求-响应模式的通信。
Service 定义了一个请求消息和一个响应消息，节点可以通过调用服务来发送请求，并等待接收响应。
服务通常用于执行较短的操作，例如查询传感器数据、请求机器人执行特定任务等。
Service 使用 rosservice 命令行工具或 ROS 客户端库（如 roscpp 或 rospy）进行调用和实现。

// 参考用库 std_srvs: http://wiki.ros.org/std_srvs
```

### Action  
    Action 是一种异步通信机制，用于节点之间执行长时间运行的任务或行为。
    Action 由三个主要组件组成：Action Goal（目标）、Action Result（结果）和Action Feedback（反馈）。
    Action Goal 是发送给执行节点的请求消息，Action Result 是执行节点发送的任务完成消息，而 Action Feedback 是执行节点在执行过程中发送的反馈消息。
    Action 通常用于执行较长时间的任务，如导航、路径规划或图像处理等。
    
    Action 使用 rostopic、rosmsg 和 rosaction 命令行工具或 ROS 客户端库进行调用和实现。

服务和动作都是基于 ROS 消息的，你需要定义自己的服务和动作消息类型来描述请求、响应、目标、结果和反馈的数据结构。

使用服务和动作可以实现节点之间的高级通信和任务协作。服务适用于简单的请求-响应模式，而动作适用于复杂的长时间运行任务。选择使用服务还是动作取决于你的应用需求和通信模式的特点。

### Topic
    Topic 是一种一对多的通信机制，其中一个节点作为发布者发布消息，而其他节点可以作为订阅者接收该消息。
    Topic 使用特定的消息类型来定义数据的结构和格式。发布者发布消息时，订阅者可以通过订阅相应的Topic来接收并处理消息。
    Topic 可以是实时或非实时的，具体取决于通信的需求和实现。
    
    Topic 基于ROS中的消息传递机制，发布者和订阅者之间通过ROS Master进行协调和连接（因此确保你的roscore是valid的).
    **SLAM任务一般采用topic作为通信机制，这可以大大提高效率，以下为官话：**
    SLAM中使用Topic的一些优势和原因：
    
    实时性和并发性: Topic提供了一种实时和并发处理数据的机制。SLAM算法通常需要实时地接收和处理来自多个传感器的数据，包括里程计数据、激光扫描数据等。通过使用Topic，不同的节点可以并行地订阅和处理这些数据，以实现高效的并发计算。
    
    可扩展性: Topic允许多个节点同时订阅同一个Topic，这使得SLAM系统可以轻松地扩展为多个功能模块。例如，你可以同时运行视觉特征提取、地图更新、位姿估计等模块，这些模块可以独立地订阅并处理里程计数据，从而提高系统的灵活性和可扩展性。
    
    数据共享和集成: 使用Topic，SLAM系统可以方便地共享数据和信息。里程计数据对于多个模块来说是一个重要的输入，通过在Topic上发布里程计数据，其他模块可以方便地订阅并使用这些数据。这种数据共享和集成有助于实现SLAM系统的整体一致性和性能提升。
    
    解耦和模块化: 使用Topic可以将SLAM系统的不同功能模块解耦，使其能够独立开发和测试。每个模块只需关注自己所需的输入数据，并通过Topic与其他模块进行通信。这种模块化设计有助于减少代码之间的依赖性，提高代码的可维护性和可重用性。

## ROS rviz visualization type: visualization_msgs

http://wiki.ros.org/rviz/DisplayTypes/Marker

## ROS tf

```c++
// Ros tf （ Transform library ），是ros 里的一个重要组成部分， 它是一个用于机器人应用中坐标系转换的库， 相信查看这份文档的人对三维刚体运动多少都有些了解， 即在三维空间中 旋转加平移可以表示任何物体在空间中的运动， 而任何此类变换都需要在坐标系中进行， 我们虽然可以用 Eigen库结合矩阵比较简单的完成此类操作，但是坐标系的变换在所难免， 比如在自瞄任务中就需要涉及到至少三个坐标系的转换（相机坐标系，世界坐标系，云台坐标系）才能配合电控完成云台控制的任务， 以下ROS tf 包括的核心概念

1. Coordinate Frames： 中文翻译（好吧，是我自己翻译的）是坐标帧，我叫他位置帧（因为感觉更好理解），每一帧都代表了物体在3D空间的方向与位置， 结合RV来说， 当调车时采用rviz可视化时你会看到多个坐标系的可视化情况，他们都可以被赋予名字，比如 “Odom”、“Camera”等等，所以一个机器人可以有多个 位置帧。
  
2. Transform: 一个Transform 表示 两个位置帧之间的旋转和平移，就像Eigen库用Rotation矩阵或者偏移向量 描述三维刚体运动一样， 你同样可以采用Eigen来完成这些操作， 但tf提供了更多更方便的操作，比如可以在rviz中将每一步可视化；并且不同坐标系之间的转化关系可以又parent frame 和 children frame 之间的差别进行推导。
 
3. tf2 ros：这是一个ros官方给的 package， 用于work with tf， 包括publisher和subscriber用来广播和监听tf的transforms
  
4. tf2 transform Broadcaster and listener: tf2_ros 的重要组成部分， transform broadcaster 用于发布 frame to another frame 的变化信息， transfrom listener 监听并缓存最新的 transform消息并参与应用。
  
5. tf_echo: 用于打印最新两帧转化的命令行工具
  
6. tf _static: TF 的一个库，用于固定不变的帧。
```

## ros and 线程

### 1. rclcpp 有关内容

```c++
// rclcpp::AsyncParametersClient 
https://docs.ros.org/en/humble/p/rclcpp/generated/classrclcpp_1_1AsyncParametersClient.html

// What: 通过同步，获取节点参数


// How: 通过同步获取节点参数

auto parameters_client = std::make_shared<rclcpp::SyncParametersClient> (node, "set param node name");

// How: 通过异步获得节点参数

auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient> (node, "set param node name");







```

### 2. Composing multiple nodes in a single process

```c++
// 1. components
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
## ros bag


## ros2 for unity

Examplified with ros2 humble here: https://github.com/RobotecAI/ros2-for-unity

## ros2 serial driver

```
// 官方文档地址如下
https://docs.ros.org/en/humble/p/serial_driver/generated/classdrivers_1_1serial__driver_1_1SerialDriver.html#class-documentation
```

### Namespace

### Class

```c++
// The most important head file is serial_driver.hpp
class SerialDriver
   explicit SerialDriver(const IoContext &ctx);
	 void init_port(const std::string &device_name, const SerialPortConfig &config);
	 std::shared_ptr<SerialPort> port() const;

class SerialPort 
  /**
  Default constructor.

		Parameters:
			ctx – [in] An IoContext object to handle threads

			device_name – [in] The name of the serial device in the OS

			serial_port_config – [in] Configuration options for the serial port
  */
   SerialPort(const IoContext &ctx, const std::string &device_name, const SerialPortConfig serial_port_config);


class SerialPortConfig:	
/**
	SerialPortConfig(uint32_t baud_rate,  flow_control, Parity parity, StopBits stop_bits)：类的构造函数，用于创建 SerialPortConfig 对象。它接受四个参数：baud_rate（波特率）、flow_control（数据流控制方式）、parity（奇偶校验方式）和 stop_bits（停止位数量）。

uint32_t get_baud_rate() const：返回配置的波特率（baud rate）值，单位为 bps（比特每秒）。

spb::baud_rate get_baud_rate_asio() const：返回配置的波特率作为 ASIO（Asynchronous I/O）对象。ASIO 是一种用于异步 I/O 操作的库。

FlowControl get_flow_control() const：返回配置的数据流控制方式（Flow Control）。

spb::flow_control::type get_flow_control_asio() const：返回配置的数据流控制方式作为 ASIO 对象。

Parity get_parity() const：返回配置的奇偶校验方式（Parity）。

spb::parity::type get_parity_asio() const：返回配置的奇偶校验方式作为 ASIO 对象。

StopBits get_stop_bits() const：返回配置的停止位数量（Stop Bits）。

spb::stop_bits::type get_stop_bits_asio() const：返回配置的停止位数量作为 ASIO 对象。
*/
		
```



### Enums

```c++
FlowControl
/**
FlowControl是一个枚举（Enum）类型，它定义在文件serial_port.hpp中，并在drivers::serial_driver命名空间中。该枚举类型用于表示串行通信（Serial Communication）中的数据流控制（Flow Control）方式。

在串行通信中，数据流控制用于确保数据的可靠传输和处理。当数据发送方和接收方的处理速度不匹配时，可能会导致数据丢失或溢出。数据流控制机制通过发送特殊的控制信号来调节数据传输的速率，以确保数据的正确接收和处理。

在FlowControl枚举中，有三个可能的值：

NONE: 表示没有数据流控制。在这种情况下，数据发送方和接收方之间没有流控制信号的交互，数据按照原始速率发送和接收。

HARDWARE: 表示硬件流控制。使用硬件流控制时，数据发送方和接收方之间使用硬件信号线（如RTS：Request to Send/CTS:Clear to send）来进行流控制。当接收缓冲区满时，数据发送方将停止发送数据，直到接收缓冲区有足够的空间接收数据。

SOFTWARE: 表示软件流控制。使用软件流控制时，数据发送方和接收方之间使用控制字符（如XON/XOFF）来进行流控制。当接收缓冲区满时，接收方发送XOFF信号给发送方，发送方停止发送数据，直到接收方发送XON信号恢复数据传输。

补充：RTS and CTS

RTS（Request to Send）：由数据接收方（通常是接收端设备）发送给数据发送方（通常是发送端设备）。当接收方的接收缓冲区准备好接收数据时，它会发送 RTS 信号给发送方，请求发送方开始传输数据。

CTS（Clear to Send）：发送方接收到 RTS 信号后，如果它的发送缓冲区也准备好发送数据，就会发送 CTS 信号给接收方。接收方收到 CTS 信号后，确认接收缓冲区已经准备好，发送方开始传输数据。

RTS/CTS 是一种硬件流控制方式，因此不需要发送特殊字符或字节来进行流控制。相比软件流控制（如 XON/XOFF），RTS/CTS 的优点在于它的响应速度更快，不会增加传输数据的字节数。

请注意，RTS/CTS 的使用需要硬件支持，即串行通信设备（如串口、USB串口转换器等）必须具备 RTS 和 CTS 信号线。在某些情况下，硬件设备可能不支持 RTS/CTS 流控制，因此在选择流控制方式时应考虑设备的兼容性。
*/

Parity
/**
`Parity`是另一个枚举（Enum）类型，同样定义在文件`serial_port.hpp`中，位于`drivers::serial_driver`命名空间中。该枚举类型用于表示串行通信（Serial Communication）中的奇偶校验（Parity）位。

在串行通信中，奇偶校验用于检测和纠正数据传输中的错误。它通过在每个数据字节后添加一个奇偶校验位来实现。奇偶校验位的值取决于数据字节中的位的总数（奇数或偶数），以及校验位的类型（奇校验或偶校验）。接收方在接收数据时会计算接收到的数据中的奇偶校验位，并与发送方发送的校验位进行比较，从而判断数据是否正确传输。

在`Parity`枚举中，有三个可能的值：

1. `NONE`: 表示不使用奇偶校验。在这种情况下，数据传输不包含奇偶校验位，数据以原始形式发送和接收，不进行校验。

2. `ODD`: 表示使用奇校验。在这种情况下，数据传输包含一个奇校验位，使得数据字节中的位总数（包括校验位）为奇数。

3. `EVEN`: 表示使用偶校验。在这种情况下，数据传输包含一个偶校验位，使得数据字节中的位总数（包括校验位）为偶数。

奇偶校验位在串行通信中是一个简单但有效的错误检测机制。使用奇偶校验可以帮助在传输过程中发现并纠正少量的数据错误，提高数据传输的可靠性。然而，奇偶校验并不能纠正所有类型的错误，对于更高级的错误检测和纠正，通常需要更复杂的校验方法。


在该项目中，我们采用的是CRC校验， 此后我们都将采用此种校验方式。
*/

StopBits
/**
`StopBits`是另一个枚举（Enum）类型，同样定义在文件`serial_port.hpp`中，位于`drivers::serial_driver`命名空间中。该枚举类型用于表示串行通信（Serial Communication）中的停止位（Stop Bits）数量。

在串行通信中，每个数据字节都由数据位（Data Bits）、奇偶校验位（如果启用了奇偶校验）和停止位组成。停止位用于标识一个数据字节的结束。在每个数据字节的后面，发送方会插入指定数量的停止位，接收方在接收数据时，根据停止位的位置来识别数据帧的边界。

在`StopBits`枚举中，有三个可能的值：

1. `ONE`: 表示使用一个停止位。在这种情况下，每个数据帧由一个数据位、一个奇偶校验位（如果启用了奇偶校验）和一个停止位组成。

2. `ONE_POINT_FIVE`: 表示使用1.5个停止位。这种情况下并不常见，通常只在特殊情况下使用。每个数据帧由一个数据位、一个奇偶校验位（如果启用了奇偶校验）和1.5个停止位组成。

3. `TWO`: 表示使用两个停止位。在这种情况下，每个数据帧由一个数据位、一个奇偶校验位（如果启用了奇偶校验）和两个停止位组成。

停止位的数量是串行通信的重要参数，它与数据传输的稳定性和可靠性密切相关。通常，大多数串行通信设备和协议使用一个停止位，但在某些特殊情况下，可能需要使用更多的停止位来满足特定设备或协议的要求。
*/
```



### Typedef

### 总结：如何使用？

```c++
// 一般采用 unique_ptr 来保存SerialDriver 对象

#include <serial_driver/serial_driver.hpp>

std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver;
std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_conifg;


/**
	device_name: Your port name like /tty/ACM0
	device_config: driver::serial_driver::SerialPortConfig: 
							包含三个部分：FlowControl, Parity, Stopbits
*/

serial_driver->init_port("device_name",device_conifg);
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

### 1.catkin_create_pkg

### 2.catkin_make

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

## rosbag

在ROS 1 中，`rosbag`是一个用于记录和回放ROS消息数据的工具, 我们可以使用`rosbag`来记录ROS话题的消息，并在稍后的时间点进行回放或分析。

下面是一些常见的`rosbag`使用示例：

1. 记录ROS话题的消息：
   ```
   rosbag record -a
   ```
   这将记录所有活动话题的消息。您还可以指定特定的话题来进行记录。

2. 记录指定话题的消息：
   ```
   rosbag record -O <bag文件名> <话题1> <话题2> ...
   ```
   使用`-O`选项指定要保存的`rosbag`文件的名称，并在后面列出要记录的话题。

3. 回放rosbag文件：
   ```
   rosbag play <bag文件名>.bag
   ```
   这将回放指定的`rosbag`文件。您可以在回放时使用`-s`选项来指定回放的开始时间，使用`-r`选项来指定回放速率。

4. 显示rosbag文件信息：
   ```
   rosbag info <bag文件名>.bag
   ```
   这将显示有关`rosbag`文件的详细信息，例如包含的话题、消息数量等。

5. 导出rosbag文件中的消息：
   ```
   rosbag export <bag文件名>.bag <输出目录>
   ```
   这将从`rosbag`文件中提取消息，并将其导出到指定的输出目录中。

这些只是`rosbag`的一些基本用法示例。您可以通过运行`rosbag --help`命令来获取更多命令选项和使用说明。此外，ROS文档中也有关于`rosbag`的详细信息，您可以查阅官方文档以获取更多帮助。

## ROS1 bug
### roscore 
	WARNING: Could not change permissions for folder [/home/ma/.ros/log/20713ec6-2522-11ee-acad-2bf8c6a50870], make sure that the parent folder has correct permissions.
	then 
 sudo chown -R ma:ma /home/ma/.ros/log/


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

