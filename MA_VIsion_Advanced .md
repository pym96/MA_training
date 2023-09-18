# C++ 部分

## attribute in the rail of struct

````c++
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


示例如下：
struct ReceivePacket
{
  uint8_t header = 0x5A;
  uint8_t detect_color : 1;  // 0-red 1-blue
  bool reset_tracker : 1;
  uint8_t reserved : 6;
  float roll;
  float pitch;
  float yaw;
  float aim_x;
  float aim_y;
  float aim_z;
  uint16_t checksum = 0;
} __attribute__((packed));
  问：当一个特定的对象被建立时，比如ReceivePacket p; p的各个变量在内存上是按结构体内的声明顺序分布的吗？
    
    
  答：
   当一个特定的对象（例如 `ReceivePacket p;`）被创建时，结构体内的成员变量在内存上是按照结构体内的声明顺序进行分布的。这意味着内存中的布局将按照在结构体中声明的顺序存储每个成员变量。

以上成员变量的声明顺序是：

```cpp
uint8_t header = 0x5A;
uint8_t detect_color : 1;  // 0-red 1-blue
bool reset_tracker : 1;
uint8_t reserved : 6;
float roll;
float pitch;
float yaw;
float aim_x;
float aim_y;
float aim_z;
uint16_t checksum = 0;
```

在内存中，`p` 对象的布局将按照上述顺序存储这些成员变量。每个成员变量将按照其对应的类型在内存中占据相应的字节大小，并按照顺序紧密地排列在一起，以构成完整的结构体对象 `p`。

但结构体的内存布局还可能受到编译器的优化和对齐方式的影响。为了确保结构体成员的正确对齐，有时编译器可能在成员之间添加填充字节，所以可以使用 `__attribute__((packed))` 这样的特性来告诉编译器不要进行额外的填充，以便更精确地控制结构体的内存布局。
````

```

## C++20 新特性

### Designated initializer

```c++
// 在C++ 20 中， .value_name = value 是一种声明类和结构体成员的一种新的方式。
Detector::LightParams l_params = {
    .min_ratio = declare_parameter("light.min_ratio", 0.1),
    .max_ratio = declare_parameter("light.max_ratio", 0.4),
    .max_angle = declare_parameter("light.max_angle", 40.0)};
//指定的初始值设定项允许您在初始化期间指定结构或类中特定成员的值，而不必按声明顺序为所有成员提供值。
```

## Why smart pointer?

```c++
// 首先要提出一个问题：当构建项目时，你是倾向于用对象管理资源还是用指针管理资源？

/**
在C++中，选择如何管理资源取决于代码的具体上下文和要求。使用对象管理和使用类对象指针都有各自的优点和权衡。让我们讨论每种方式：

1. **对象管理（自动存储期）**：
   - 当直接创建对象（例如：`MyClass myObject;`）时，它们会自动由自动存储期管理。
   - 这意味着当对象超出其作用域时（例如：创建它的块结束时），对象会自动销毁，其资源会被释放。
   - 这种方式更安全，有助于防止资源泄漏。它减少了内存泄漏的风险，因为资源在不再需要时会被自动清理。
   - 这种方式通常适用于简单的场景，不需要将对象的所有权转移给代码的其他部分。

2. **类对象指针（动态存储期）**：
   - 当使用类对象指针（例如：`MyClass* ptr = new MyClass;`）时，您需要显式地管理对象的生命周期。
   - 这种方式更加灵活，允许您动态地创建和销毁对象，以便控制资源的分配和释放。
   - 它允许更复杂的所有权场景，比如将对象的所有权传递给代码的其他部分（例如：从函数返回动态分配的对象）。
   - 然而，如果处理不当，这样做会增加内存泄漏和资源管理不当的风险。开发人员必须记得在不再需要内存时释放它（使用`delete`），以避免内存泄漏, 作为开发者，就个人经历而言，当代码量扩大时很难记住new 出来的对象是否被释放、释放的时机是否正确以及是否被释放了多次。
   - 智能指针（例如：`std::unique_ptr`、`std::shared_ptr`）可以用于更安全地管理对象生命周期，而不是使用原始指针。

**哪种方式更好？**
- 如果可以使用对象管理（自动存储期），通常是更安全和更简单的选择。它减少了资源泄漏的风险，通常更易于理解。
- 然而，有些情况下需要或受益于动态对象管理（使用指针），例如当需要动态地控制对象的生命周期或者实现更复杂的所有权场景时。

在现代C++中，当需要动态内存管理时，鼓励使用智能指针（`std::unique_ptr`和`std::shared_ptr`）。智能指针有助于更安全地管理对象的生命周期和所有权，减少了内存泄漏和资源管理不当的风险。

总结：在 RM 的视觉组任务中，无论是否采用ros 框架，多线程都是绕不开的话题，相机资源和串口资源在多个线程之间的转化必然需要指针这种容易转让的结构来胜任，而为了更好的安全性，smart pointer就是不二之选。
*/
```

## std::bind with std::place_holder

~~~c++
/**
`std::bind` 和 `std::placeholders` 是 C++11 中引入的功能，它们通常用于创建函数对象（Function Objects），也称为函数包装器（Function Wrappers）。`std::bind` 允许我们将函数与其参数绑定在一起，形成一个新的可调用对象。`std::placeholders` 则是用于占位参数，表示在调用绑定的函数对象时，某些参数是预留的，需要在实际调用时再提供。

首先，先来了解一下 `std::bind` 的用法：

```cpp
*/

#include <functional> // 包含头文件以使用 std::bind 和 std::placeholders

// 示例函数，接受两个整数参数并返回它们的和
int add(int a, int b) {
    return a + b;
}

int main() {
    // 使用 std::bind 将函数 add 与参数 10 绑定
    auto add_10 = std::bind(add, 10, std::placeholders::_1);

    // 调用 add_10，并传入一个参数作为第二个参数
    int result = add_10(5); // 此时相当于调用 add(10, 5)
    std::cout << "Result: " << result << std::endl; // 输出 Result: 15

    return 0;
}


/**
在上面的例子中，我们使用 `std::bind` 创建了一个新的函数对象 `add_10`，它是函数 `add` 与参数 `10` 绑定在一起的结果。我们使用 `std::placeholders::_1` 来表示第一个预留的参数位置。当我们调用 `add_10(5)` 时，实际上是调用了 `add(10, 5)`，返回结果为 15。

我们也可以绑定多个参数：

```cpp
*/

#include <functional>
#include <iostream>

void print_sum(int a, int b, int c) {
    std::cout << "Sum: " << (a + b + c) << std::endl;
}

int main() {
    auto print_sum_abc = std::bind(print_sum, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

    // 调用 print_sum_abc，并传入三个参数
    print_sum_abc(10, 20, 30); // 输出 Sum: 60

    return 0;
}

/**
在这个例子中，我们使用 `std::bind` 将函数 `print_sum` 与三个参数绑定在一起，然后通过 `print_sum_abc(10, 20, 30)` 调用，实际上是调用了 `print_sum(10, 20, 30)`，输出结果为 60。

通过 `std::bind` 和 `std::placeholders`，我们可以方便地创建灵活的函数对象，并在稍后提供参数。这在某些场景下非常有用，特别是在设计回调函数时。然而，在现代的 C++ 中，推荐使用 Lambda 表达式来替代 `std::bind`，因为 Lambda 表达式更加灵活、清晰，并且在性能上通常更好。
*/
~~~

## std::vector resize() and reserve()

```c++
reserve() 用于预留内存容量，不改变大小。
resize() 用于改变 vector 的大小，可以增加或减少元素的数量。
  
 `resize()` 会改变 vector 的大小，并且可能会导致内存分配和重新分配。当你使用 `resize()` 增加 vector 的大小时，如果新的大小超过了当前容量，vector 可能会重新分配内存以容纳更多的元素，这可能会导致内存重新分配和拷贝。如果新的大小小于当前容量，vector 不会重新分配内存，但超出新大小的元素会被删除。

总结：`resize()` 可以改变 vector 的大小，但可能会影响内存的分配和使用情况。如果预先分配足够的内存而不改变 vector 的大小，可以使用 `reserve()` 函数。
  
```

# Docker
## Start with docker 
https://www.digitalocean.com/community/tutorials/how-to-install-and-use-docker-on-ubuntu-22-04

```c++
// 基础的命令：
docker images
docker imaegs rm image_name
docker run -it image_name
// image运行成为container后提交更改命令：
docker ps （查看当前运行的container name）
docker commit -m "Some description" container_name
//对于container的命令
docker ps -a （查看所有容器名）
docker start container_name
docker stop container_name
docker exec -it container_name /bin/bash // 运行某个容器
  
// 将当前pc的文件或者文件夹拷贝到container中
 docker ps <PC_path> <container_name>:destination_path 
```




# ROS_CONTROL 文档
![image](https://github.com/pym96/MA_training/assets/105438207/6a785eca-ad86-45f6-a913-f29453a69004)

![image](https://github.com/pym96/MA_training/assets/105438207/ab893ef5-3fce-4a8b-b908-2fc2311d33bf)

![image](https://github.com/pym96/MA_training/assets/105438207/f4651975-182d-4b92-89ab-04b4a5ce0851)

![image](https://github.com/pym96/MA_training/assets/105438207/07975db2-6e63-4910-b90c-9dc1424482aa)

![image](https://github.com/pym96/MA_training/assets/105438207/5f4d03be-a0af-402c-82d6-959328e4506c)

### 开环控制
![image](https://github.com/pym96/MA_training/assets/105438207/c9d07bd7-7170-4f2f-8441-ef91cd8443fa)

### 闭环控制
![image](https://github.com/pym96/MA_training/assets/105438207/0b3eef61-798e-4667-a2e5-42077c977ec2)

![image](https://github.com/pym96/MA_training/assets/105438207/3b8b8e76-8dfb-40eb-9433-629966ba0ecd)

### Demo in real 
![image](https://github.com/pym96/MA_training/assets/105438207/6ef3e1ec-4bbd-4b95-a5b4-a94ae6745454)

### Camera in ros 
![image](https://github.com/pym96/MA_training/assets/105438207/6a8d58de-4eaf-4076-9e5d-e787848b1a9e)




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

## IMU (Inertial measurement unit): In robotics, [odometry](https://en.wikipedia.org/wiki/Odometry) is about using data from sensors to estimate the change in a robot’s  position, orientation, and velocity over time relative to some [point (e.g. x=0, y=0, z=0)](https://www.ros.org/reps/rep-0105.html#coordinate-frames). 

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

## Control theory

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

```c++
// 这部分代码示例请参考 github 中的 ma_train_for_ros 仓库， 经过我本人的学习来看，此部分较为简单，应该只有以下这部分需要理解一下：

在ROS 2中，`visualization_msgs::msg::Marker` 消息有三种不同的动作类型：

1. `visualization_msgs::msg::Marker::ADD`：该动作类型用于向可视化场景中添加一个新的标记（Marker）。如果已存在具有相同`id`的标记，它将被新的标记替换。

2. `visualization_msgs::msg::Marker::MODIFY`：该动作类型用于修改可视化场景中的现有标记。标记的`id`必须与现有标记的`id`匹配，该动作才能生效。例如，如果你想更改已显示标记的颜色、位置或方向，可以发布一个带有`MODIFY`动作和相应`id`的新标记消息。

3. `visualization_msgs::msg::Marker::DELETE`：该动作类型用于从可视化场景中删除现有标记。同样，标记的`id`必须与现有标记的`id`匹配，该标记才会被删除。这在你想从场景中删除一个标记时很有用，也许因为它不再相关或不需要显示。

在RViz 2中，你可以通过将`visualization_msgs::msg::Marker`消息发布到`visualization_marker`主题来使用这些动作类型。根据你使用的动作类型，RViz 2将添加一个新标记、修改现有标记或从可视化场景中删除一个标记。这使得你可以根据应用程序的需求动态更新和管理在RViz 2中显示的可视化效果。
  
  
visualization_msgs::msg::Marker消息的三种动作类型（ADD、MODIFY和DELETE），它们在操作上的差异如下：

ADD动作：使用ADD动作时，你无需指定id，它会直接添加一个新的标记到可视化场景中。如果已经存在具有相同id的标记，它会被新的标记替换。

MODIFY动作：使用MODIFY动作时，你需要指定要修改的现有标记的id。只有标记的id匹配现有标记的id时，才能对该标记进行修改。你可以通过发布带有MODIFY动作和相应id的新标记消息来更改标记的属性，例如位置、颜色或大小。

DELETE动作：使用DELETE动作时，同样需要指定要删除的现有标记的id。只有标记的id匹配现有标记的id时，该标记才会从可视化场景中删除。

所以，总结起来：

ADD动作用于添加新的标记，无需id。
MODIFY动作用于修改现有标记，需要指定相应的id。
DELETE动作用于删除现有标记，也需要指定相应的id。
  
// 对于makrer array 
关于marker_array 的理解，可将其理解为一个类型为 marker 的 vector，它可以管理多个 marker 并做到同时发布。
```



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

## Cv_bridge

![image-20230727160910693](/Users/dan/Library/Application Support/typora-user-images/image-20230727160910693.png)

```c++
// 具体内容请参考文档 http://wiki.ros.org/cv_bridge
```

## QOS

```c++
// qos (quality of service): 在ros中可以简单的理解为不同节点之间通信的要求，比如best effort, default 


/**
在这个例子中，我们使用 `std::bind` 将函数 `print_sum` 与三个参数绑定在一起，然后通过 `print_sum_abc(10, 20, 30)` 调用，实际上是调用了 `print_sum(10, 20, 30)`，输出结果为 60。

通过 `std::bind` 和 `std::placeholders`，我们可以方便地创建灵活的函数对象，并在稍后提供参数。这在某些场景下非常有用，特别是在设计回调函数时。然而，在现代的 C++ 中，推荐使用 Lambda 表达式来替代 `std::bind`，因为 Lambda 表达式更加灵活、清晰，并且在性能上通常更好。
*/




// 用法 
#include <rclcpp/qos.hpp>

// 文档地址如下：
https://docs.ros.org/en/ros2_packages/rolling/api/rclcpp/generated/classrclcpp_1_1SensorDataQoS.html


// 自定义qos blog 如下：
https://blog.csdn.net/sph123s/article/details/108223669

```



## rcl_interfaces

```c++
// 描述：The ros client library common interfaces. 包含 ROS 客户端库并将在后台使用的消息和服务来传达更高级别的概念，例如参数。

// rcl_interfaces::msg::Parameterdescriptor， 本质上是一个定义在ROS2 中间层

// 文档地址： https://docs.ros.org/en/humble/Concepts/Basic/About-Interfaces.html

// 如何使用

// # 1. 如果不用, 例子如下：

#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv){
  
  rclcpp::init(argc, argv);
  
  auto node = rclcpp::Node::make_shared("my_node");
 
  // Declare the parameter with an initial node
  int parameter = delcare_parameter<int>("my_parameter",666);
  
   // Check and set limitations on the parameter value
    if (my_parameter < 0) {
        RCLCPP_WARN(node->get_logger(), "my_parameter cannot be negative. Setting to 0.");
        my_parameter = 0;
    } else if (my_parameter > 100) {
        RCLCPP_WARN(node->get_logger(), "my_parameter cannot exceed 100. Setting to 100.");
        my_parameter = 100;
    }

    RCLCPP_INFO(node->get_logger(), "my_parameter: %d", my_parameter);
  
  rclcpp::shutdown();
  
  return 0;
}

// 可以看到，代码在加了约束之后变得很臃肿， 为了代码的整洁，因此采用 rcl_interfaces::msgParamtertDescriptor 来使用函数进行约束， 以下为例子：

  rcl_interfaces::msg::ParameterDescriptor param_desc; // 声明对象
  param_desc.integer_range.resize(1); // resize inter_range 数组
  param_desc.integer_range[0].step = 1; // 确保每个 parameter 都是1的倍数
  param_desc.integer_range[0].from_value = 0; // 最小值为0
  param_desc.integer_range[0].to_value = 255; // 最大值为 255
  int binary_thres = declare_parameter("binary_thres", 160, param_desc);

  param_desc.description = "0-RED, 1-BLUE"; // 参数描述以供参考
  param_desc.integer_range[0].from_value = 0; // 最小值
  param_desc.integer_range[0].to_value = 1; // 最大值
  auto detect_color = declare_parameter("detect_color", RED, param_desc);

// 你可以能对integer_range.resize(1) 有疑问， 但只要记住，如果要增加范围约束就要使用它，反例如下：
rcl_interfaces::msg::ParameterDescriptor param_desc;
param_desc.description = "This parameter has no constraints.";
auto my_parameter = declare_parameter("my_parameter", 42, param_desc);

```


## ROS with smart pointer

### ROS1 

```c++
/**
在ROS 1中，使用的C++客户端库是roscpp，它与ROS 2的rclcpp不同。在roscpp中，并没有像ROS 2中rclcpp那样普遍使用`std::shared_ptr`。在ROS 1的roscpp中，资源管理主要使用裸指针（raw pointers）和boost库中的`boost::shared_ptr`。

以下是在ROS 1中常见的资源管理方式：

1. **ROS节点（Node）**:
   - 在ROS 1中，节点通常使用裸指针来管理。例如，通过`ros::NodeHandle`创建的节点对象通常不使用智能指针进行管理。

2. **ROS订阅者（Subscriber）和发布者（Publisher）**:
   - ROS 1中的订阅者和发布者通常也使用裸指针或者boost库中的`boost::shared_ptr`进行管理。例如，通过`ros::Subscriber`和`ros::Publisher`创建的对象通常使用这些方法来管理。
*/
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <boost/shared_ptr.hpp>

class MyNode {
public:
  MyNode() {
    // Create a publisher with boost::shared_ptr
    publisher_ = boost::make_shared<ros::Publisher>(
      nh_.advertise<std_msgs::String>("topic_name", 10)
    );

    // Create a subscriber with boost::shared_ptr
    subscriber_ = boost::make_shared<ros::Subscriber>(
      nh_.subscribe("topic_name", 10, &MyNode::messageCallback, this)
    );
  }

  // Callback function
  void messageCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("Received message: %s", msg->data.c_str());
  }

private:
  ros::NodeHandle nh_;
  // Publisher with boost::shared_ptr
  boost::shared_ptr<ros::Publisher> publisher_;

  // Subscriber with boost::shared_ptr
  boost::shared_ptr<ros::Subscriber> subscriber_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "my_node");

  // Create a node
  MyNode node;

  ros::spin();

  return 0;
}


/**
3. **回调函数**:
   - ROS 1中的回调函数也通常使用裸指针。例如，订阅者回调函数使用的参数是指向ROS消息的裸指针。

4. **节点执行器（Node Handle）**:
   - ROS 1的节点执行器使用`ros::NodeHandle`对象，它在创建时不使用智能指针。

需要注意的是，ROS 1是在ROS 2之前的版本，因此在ROS 1中可能没有像ROS 2中那样普遍使用现代C++特性和智能指针。ROS 1仍然使用了boost库来提供智能指针的功能（`boost::shared_ptr`），但它没有像ROS 2中的`std::shared_ptr`那样成为标准C++特性。
*/


/**
总结：
使用ROS 1，并且想要使用智能指针来更方便地管理资源，可以考虑使用boost库中的`boost::shared_ptr`。如果您在ROS 2中进行开发，rclcpp库已经采用了现代C++特性，包括`std::shared_ptr`，这使得资源管理更加便捷。
*/
```



### ROS2 

```c++
/**
在ROS 2中，rclcpp库提供了许多使用`std::shared_ptr`的类和对象，这是因为ROS 2中的C++客户端库（rclcpp）采用了现代C++编程范式，尤其是智能指针。使用`std::shared_ptr`可以更方便地管理资源的生命周期，避免内存泄漏和悬空指针等问题。以下是一些常见情况下使用`std::shared_ptr`的示例：
*/

/**
1. **ROS 2节点（Node）**:
   - 在ROS 2中，使用`rclcpp::Node::SharedPtr`来管理ROS 2节点对象的生命周期。这样做可以确保节点在不再需要时正确释放资源，用起来像java 虚拟机那样舒畅。
*/


/**
2. **ROS 2订阅者（Subscriber）和发布者（Publisher）**:
   - 通常使用`rclcpp::Subscription`和`rclcpp::Publisher`类的`SharedPtr`版本来管理订阅者和发布者的生命周期。这样做可以避免因订阅者和发布者的生命周期不匹配而导致的问题。
*/

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>

class MyNode : public rclcpp::Node {
public:
  MyNode() : Node("my_node") {
    // Create a publisher with std::shared_ptr
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic_name", 10);

    // Create a subscriber with std::shared_ptr
    subscriber_ = this->create_subscription<std_msgs::msg::String>(
      "topic_name",
      10,
      std::bind(&MyNode::messageCallback, this, std::placeholders::_1)
    );
  }

private:
  // Publisher with std::shared_ptr
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  // Subscriber with std::shared_ptr
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;

  // Callback function using std::shared_ptr as the parameter
  void messageCallback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(get_logger(), "Received message: %s", msg->data.c_str());
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  // Create a node with std::shared_ptr
  auto node = std::make_shared<MyNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}


/**
3. **回调函数**:
   - 当您将回调函数与ROS 2订阅者或服务端（Service）相关联时，可以使用`std::shared_ptr`作为回调函数的参数类型。这样做可以确保回调函数中访问的资源在合适的时候被正确释放。
*/

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>

// Callback function type using std::shared_ptr as the parameter
void messageCallback(const std_msgs::msg::String::SharedPtr msg) {
  RCLCPP_INFO(rclcpp::get_logger("subscriber_node"), "Received message: %s", msg->data.c_str());
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("subscriber_node");

  // Create a subscriber and associate the callback function with it
  auto subscriber = node->create_subscription<std_msgs::msg::String>(
    "topic_name",
    10, // Queue size
    messageCallback // Callback function
  );

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}


/**
4. **节点执行器（Executor）**:
   - 节点执行器是用于执行ROS 2节点的主要循环的实用程序。`rclcpp::executors::SingleThreadedExecutor`和`rclcpp::executors::MultiThreadedExecutor`等执行器类通常使用`std::shared_ptr`来管理节点和回调函数。
*/


/** 总结：
使用`std::shared_ptr`可以使资源的管理更容易，特别是在复杂的ROS 2应用程序中。智能指针的优点包括：

- 自动处理资源的生命周期，避免手动释放资源和内存泄漏。
- 提供简单的共享所有权机制，有助于在多个地方共享资源。
- 可以很好地处理异步操作，特别是在多线程环境下。
*/

```

## Sensor_msgs/msg

### Check msg detail in ros1
(base) dan@dan-RedmiBook-Pro-14S:~$ rosmsg show sensor_msgs/Imu
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Quaternion orientation
  float64 x
  float64 y
  float64 z
  float64 w
float64[9] orientation_covariance
geometry_msgs/Vector3 angular_velocity
  float64 x
  float64 y
  float64 z
float64[9] angular_velocity_covariance
geometry_msgs/Vector3 linear_acceleration
  float64 x
  float64 y
  float64 z
float64[9] linear_acceleration_covariance



### Sensor_msgs/msg/camera_info.hpp

```c++
# This message defines meta information for a camera. It should be in a
# camera namespace on topic "camera_info" and accompanied by up to five
# image topics named:
#
#   image_raw - raw data from the camera driver, possibly Bayer encoded
#   image            - monochrome, distorted
#   image_color      - color, distorted
#   image_rect       - monochrome, rectified
#   image_rect_color - color, rectified
#
# The image_pipeline contains packages (image_proc, stereo_image_proc)
# for producing the four processed image topics from image_raw and
# camera_info. The meaning of the camera parameters are described in
# detail at http://www.ros.org/wiki/image_pipeline/CameraInfo.
#
# The image_geometry package provides a user-friendly interface to
# common operations using this meta information. If you want to, e.g.,
# project a 3d point into image coordinates, we strongly recommend
# using image_geometry.
#
# If the camera is uncalibrated, the matrices D, K, R, P should be left
# zeroed out. In particular, clients may assume that K[0] == 0.0
# indicates an uncalibrated camera.

#######################################################################
#                     Image acquisition info                          #
#######################################################################

# Time of image acquisition, camera coordinate frame ID
Header header    # Header timestamp should be acquisition time of image
                 # Header frame_id should be optical frame of camera
                 # origin of frame should be optical center of camera
                 # +x should point to the right in the image
                 # +y should point down in the image
                 # +z should point into the plane of the image


#######################################################################
#                      Calibration Parameters                         #
#######################################################################
# These are fixed during camera calibration. Their values will be the #
# same in all messages until the camera is recalibrated. Note that    #
# self-calibrating systems may "recalibrate" frequently.              #
#                                                                     #
# The internal parameters can be used to warp a raw (distorted) image #
# to:                                                                 #
#   1. An undistorted image (requires D and K)                        #
#   2. A rectified image (requires D, K, R)                           #
# The projection matrix P projects 3D points into the rectified image.#
#######################################################################

# The image dimensions with which the camera was calibrated. Normally
# this will be the full camera resolution in pixels.
uint32 height
uint32 width

# The distortion model used. Supported models are listed in
# sensor_msgs/distortion_models.h. For most cameras, "plumb_bob" - a
# simple model of radial and tangential distortion - is sufficent.
string distortion_model

# The distortion parameters, size depending on the distortion model.
# For "plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3).
float64[] D

# Intrinsic camera matrix for the raw (distorted) images.
#     [fx  0 cx]
# K = [ 0 fy cy]
#     [ 0  0  1]
# Projects 3D points in the camera coordinate frame to 2D pixel
# coordinates using the focal lengths (fx, fy) and principal point
# (cx, cy).
float64[9]  K # 3x3 row-major matrix

# Rectification matrix (stereo cameras only)
# A rotation matrix aligning the camera coordinate system to the ideal
# stereo image plane so that epipolar lines in both stereo images are
# parallel.
float64[9]  R # 3x3 row-major matrix

# Projection/camera matrix
#     [fx'  0  cx' Tx]
# P = [ 0  fy' cy' Ty]
#     [ 0   0   1   0]
# By convention, this matrix specifies the intrinsic (camera) matrix
#  of the processed (rectified) image. That is, the left 3x3 portion
#  is the normal camera intrinsic matrix for the rectified image.
# It projects 3D points in the camera coordinate frame to 2D pixel
#  coordinates using the focal lengths (fx', fy') and principal point
#  (cx', cy') - these may differ from the values in K.
# For monocular cameras, Tx = Ty = 0. Normally, monocular cameras will
#  also have R = the identity and P[1:3,1:3] = K.
# For a stereo pair, the fourth column [Tx Ty 0]' is related to the
#  position of the optical center of the second camera in the first
#  camera's frame. We assume Tz = 0 so both cameras are in the same
#  stereo image plane. The first camera always has Tx = Ty = 0. For
#  the right (second) camera of a horizontal stereo pair, Ty = 0 and
#  Tx = -fx' * B, where B is the baseline between the cameras.
# Given a 3D point [X Y Z]', the projection (x, y) of the point onto
#  the rectified image is given by:
#  [u v w]' = P * [X Y Z 1]'
#         x = u / w
#         y = v / w
#  This holds for both images of a stereo pair.
float64[12] P # 3x4 row-major matrix


#######################################################################
#                      Operational Parameters                         #
#######################################################################
# These define the image region actually captured by the camera       #
# driver. Although they affect the geometry of the output image, they #
# may be changed freely without recalibrating the camera.             #
#######################################################################

# Binning refers here to any camera setting which combines rectangular
#  neighborhoods of pixels into larger "super-pixels." It reduces the
#  resolution of the output image to
#  (width / binning_x) x (height / binning_y).
# The default values binning_x = binning_y = 0 is considered the same
#  as binning_x = binning_y = 1 (no subsampling).
uint32 binning_x
uint32 binning_y

# Region of interest (subwindow of full camera resolution), given in
#  full resolution (unbinned) image coordinates. A particular ROI
#  always denotes the same window of pixels on the camera sensor,
#  regardless of binning settings.
# The default setting of roi (all values 0) is considered the same as
#  full resolution (roi.width = width, roi.height = height).
RegionOfInterest roi

  
// Compact defination
std_msgs/Header header
uint32 height
uint32 width
string distortion_model
float64[] D
float64[9] K
float64[9] R
float64[12] P
uint32 binning_x
uint32 binning_y
sensor_msgs/RegionOfInterest roi


// 用来保存相机的基本参数比如：相机的extrinsic 矩阵和intrinsic矩阵和 distortion paramters 
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

class CameraNode : public rclcpp::Node {
public:
  CameraNode() : Node("camera_node") {
    // Create a publisher for CameraInfo message
    publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);

    // Create a timer to publish CameraInfo every 1 second
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&CameraNode::publishCameraInfo, this)
    );

    // Create a subscriber for CameraInfo message
    subscriber_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "camera_info",
      10,
      std::bind(&CameraNode::cameraInfoCallback, this, std::placeholders::_1)
    );
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;

  void publishCameraInfo() {
    // Create a CameraInfo message
    auto camera_info_msg = std::make_shared<sensor_msgs::msg::CameraInfo>();

    // Populate the CameraInfo message with some dummy data
    camera_info_msg->header.stamp = this->now();
    camera_info_msg->height = 480;
    camera_info_msg->width = 640;

    // Publish the CameraInfo message
    publisher_->publish(camera_info_msg);
  }

  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    // Callback function to process incoming CameraInfo messages
    // Here you can access the data in the received CameraInfo message
    RCLCPP_INFO(get_logger(), "Received CameraInfo message with height: %d and width: %d", msg->height, msg->width);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  // Create a node
  auto node = std::make_shared<CameraNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}

```

## Image_transport in ros

```c++
// 在ROS（机器人操作系统）中，image_transport 是一个用于图像传输的包，它提供了一种优化的图像传输方式，旨在减少图像传输的延迟和带宽占用。

image_transport 提供了一种在ROS节点之间传输图像数据的机制，支持不同的传输方式（Transport）以适应不同的需求。传输方式可以在配置文件中指定，允许开发人员根据系统需求选择适当的传输策略。常见的传输方式包括 raw、compressed、theora 等。

以下是一些 image_transport 的重要概念和用法：

传输方式（Transport）： image_transport 允许用户选择图像传输的方式。每种传输方式都有其优点和限制，例如 compressed 传输可以减小带宽占用，但会增加处理时间。不同传输方式适用于不同的应用场景。

ImageTransport 类： image_transport 包提供了 ImageTransport 类，它是用于创建图像传输对象的工具。你可以使用 ImageTransport 类创建发布者和订阅者，以便在节点之间传输图像数据。

图像消息格式： image_transport 支持将图像消息格式进行压缩，从而减小传输的数据量。这对于在带宽受限的情况下特别有用。

图像话题： 用 image_transport创建图像话题来发布和接收图像数据。这些话题可以用于在ROS节点之间传递图像信息。
  
// 如何更改传输方式？
更改传输方式（例如 `Raw`、`Compressed`、`Theora` 等）通常通过修改话题名称来实现。在 `image_transport` 中，不同的传输方式使用不同的话题名称后缀来标识。

传输方式和话题名称的对应关系如下：
- `raw` 传输方式：在话题名称后添加 `_raw` 后缀，例如 `"camera_image_raw"`。
- `compressed` 传输方式：在话题名称后添加 `_compressed` 后缀，例如 `"camera_image_compressed"`。
- `theora` 传输方式：在话题名称后添加 `_theora` 后缀，例如 `"camera_image_theora"`。

通过在话题名称中添加不同的后缀，你可以在不同的传输方式之间进行切换，从而实现不同的数据传输策略，以适应不同的带宽和处理需求。

举个例子，如果你想从 `compressed` 传输方式切换到 `theora` 传输方式，只需将发布者和订阅者的话题名称中的后缀从 `_compressed` 改为 `_theora` 即可，其他部分的代码保持不变。
  
// 为什么要用 image_transport?
在ROS中，`Image` 消息的传输最好与 `image_transport` 结合使用，主要是出于以下几个原因：

1. **减小带宽和处理负担：** 图像数据通常具有较大的数据量，特别是在高分辨率或高帧率的情况下。使用 `image_transport` 可以选择不同的传输方式（如 `compressed` 或 `theora`），通过压缩图像数据来减小带宽占用，从而降低通信和处理的负担。

2. **实时性：** 有些传输方式在传输图像数据时能够提供更低的延迟，特别是在网络带宽受限的情况下。这对于需要实时图像数据的应用非常重要，如机器人控制、视觉导航等。

3. **可扩展性：** `image_transport` 提供了一种灵活的机制，可以根据实际需求选择不同的传输方式。这样，你可以根据应用的性能需求、硬件资源和通信环境来选择最适合的传输方式。

4. **与可视化工具的兼容性：** `image_transport` 与ROS的可视化工具（如RViz）集成得很好，可以在这些工具中方便地显示图像数据，以便进行调试和可视化分析。

5. **多平台支持：** `image_transport` 为不同的图像传输方式提供了统一的接口，这样你可以在不同的硬件平台和通信环境中使用相同的代码。

总结： `image_transport` 为在ROS中传输图像数据提供了一种更加灵活、高效和实时的方式，使图像数据在不同的应用场景中能够更好地满足需求。
```


# ROS2知识

## Ament 包工具

### C++ 工具

```c++
 // 同 ROS1 中的 catkin, ROS2 中也有自己的一套包管理工具， 叫做 ament

```c++
// 同 ROS1 中的 catkin, ROS2 中也有自己的一套包管理工具， 叫做 ament


// C++ 中使用 ament 方便构建的一些工具
auto pkg_path = ament_index_cpp::get_package_share_directory("your package name");
```


### Cmake工具

```makefile
# 当你在你的包中使用ament_export_dependencies()时，你正在告诉ROS 2构建系统和依赖管理工具，你的包需要依赖于这些指定的其他包，作用如下：

# 传递依赖信息： 当你的包作为依赖项被其他ROS 2包使用时，通过使用ament_export_dependencies()，你的包会将它所依赖的其他包的信息传递给其他包。这样，当其他包构建时，它们会知道它们需要满足你的包的依赖关系。


# 链接依赖库： 通过在ament_export_dependencies()中指定依赖包，你的包在编译时会链接到这些依赖包的库，以确保你的包能够正确地使用其中定义的功能和消息类型。

ament_export_dependencies(ament_cmake rclcpp std_msgs visualization_msgs)


```




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




## Rclcpp::ParameterEventHandler 监听参数的动态变化

```c++
// ROS1 中，paramter参数机制无法实现动态监控（需要配合专门的动态机制），比如正在使用的参数被其他节点改变了，如果不重新查询的话就无法确定改变后的值，ROS1的例子如下：

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <your_package_name/YourConfig.h>  // Replace with your dynamic reconfigure message type

// Callback function for dynamic reconfigure
void dynamicReconfigureCallback(your_package_name::YourConfig &config, uint32_t level)
{
    ROS_INFO("Received new parameter value: %d", config.your_parameter_name);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dynamic_param_demo");
    ros::NodeHandle nh;

    // Create dynamic reconfigure server
    dynamic_reconfigure::Server<your_package_name::YourConfig> server;
    dynamic_reconfigure::Server<your_package_name::YourConfig>::CallbackType f;

    f = boost::bind(&dynamicReconfigureCallback, _1, _2);
    server.setCallback(f);

    ros::spin();

    return 0;
}

// 但在ros2 中，由于引入了rclcpp::ParamterEvenHandler 机制，我们无须动态监控，只需给parameter改变时加一个回调函数即可。
 // Debug param change moniter
  std::shared_ptr<rclcpp::ParameterEventHandler> debug_param_sub_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> debug_cb_handle_;

  debug_param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
  debug_cb_handle_ =
    debug_param_sub_->add_parameter_callback("debug", [this](const rclcpp::Parameter & p) {
      debug_ = p.as_bool();
      debug_ ? createDebugPublishers() : destroyDebugPublishers();
    });


```


# ROS 1 知识
## So, what's the difference in packge.xml and Cmake in ros1 project when config the env
In a ROS1 project, both `CMakeLists.txt` and `package.xml` play crucial roles, but they serve different purposes, and they impact the build and deployment process in different ways. Let's break down the differences:

### `package.xml`

- **Purpose**: This is the package manifest file. It provides metadata about the package (like its name, version, description, maintainer, license) and its dependencies.
  
- **Dependencies**:
  - `build_depend`: Dependencies required to compile the code in this package.
  - `build_export_depend`: Dependencies required for downstream packages to build against this package.
  - `exec_depend`: Dependencies required to run the code in this package.
  - `test_depend`: Dependencies required to test the package.
  - And there are a few more specialized dependency tags.
  
- **Use Cases**:
  - It's used by ROS tools like `rosdep` to ensure that all required dependencies are installed.
  - `bloom` uses it to generate Debian packages.
  - It provides essential metadata when the package is released to the ROS build farm.

### `CMakeLists.txt`

- **Purpose**: This is essentially a script for CMake, a cross-platform build system. Here, you specify how your package should be built: which files to compile, which targets to link against, etc.
  
- **Key Commands**:
  - `find_package`: Search for other CMake or Catkin packages. It ensures required libraries and headers are available.
  - `add_executable`: Define a new build target that will be an executable.
  - `add_library`: Define a new build target that will be a library.
  - `target_link_libraries`: Specify which libraries an executable/library should link against.
  - `catkin_package`: Define properties of this package that are required for other packages to build/run against it.
  
- **Use Cases**:
  - This is where you specify how to build your ROS nodes, libraries, and tests.
  - `catkin_make` or `catkin build` uses this file to build your ROS package.
  
### In Context

While both files specify dependencies, they are used in different parts of the ROS build and deployment process:

- When you're developing and building your package locally, the build system (`catkin`) mainly relies on `CMakeLists.txt` to know what to build and how to link things together.

- When you're releasing your package, or when someone else is trying to build/run your package, the dependencies in `package.xml` become essential. They ensure that all the necessary dependencies are present. 

In conclusion, while there's some redundancy between the two files, they're both essential for different parts of the ROS package lifecycle. Ensure you maintain consistency between them, especially regarding dependencies, to avoid any build or deployment issues.

## ROS1 实践知识我写到了 github ma_train_for_ros 里，包括Cmake怎么写，自定义可运行节点怎么写，launch怎么写等等，里边也有 .md文件介绍，感兴趣的可以自己去看

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

## ros cv bridge
To publish an OpenCV `cv::Mat` image to a ROS topic, you can use the `cv_bridge` package which provides a bridge between OpenCV and ROS image messages. Here's a step-by-step guide:

1. **Install Dependencies**:
   Make sure you have `cv_bridge` and `image_transport` installed. You can install them using:
   ```bash
   sudo apt-get install ros-<your-ros-version>-cv-bridge
   sudo apt-get install ros-<your-ros-version>-image-transport
   ```

2. **Include Necessary Headers**:
   In your code, you'll need to include:
   ```cpp
   #include <ros/ros.h>
   #include <image_transport/image_transport.h>
   #include <cv_bridge/cv_bridge.h>
   #include <sensor_msgs/Image.h>
   ```

3. **Initialize ROS and Image Transport**:
   ```cpp
   ros::init(argc, argv, "image_publisher");
   ros::NodeHandle nh;
   image_transport::ImageTransport it(nh);
   image_transport::Publisher pub = it.advertise("/image_raw", 1);
   ```

4. **Convert and Publish**:
   Here's a basic way to convert a `cv::Mat` image to a ROS message and publish it:
   ```cpp
   cv::Mat image = cv::imread("path_to_image.jpg", cv::IMREAD_COLOR); // Read an image from file

   // Convert the image to a sensor_msgs/Image message
   sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

   pub.publish(msg);
   ros::spinOnce();
   ```

5. **Update CMakeLists.txt**:
   You'll also need to ensure that you've added the necessary dependencies to your `CMakeLists.txt`:
   ```cmake
   find_package(catkin REQUIRED COMPONENTS
     roscpp
     image_transport
     cv_bridge
     # other packages
   )
   ```

   And also ensure you link against the necessary libraries:
   ```cmake
   target_link_libraries(your_target ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
   ```

6. **Run**:
   Make sure to source your ROS workspace and then run your node.

Remember that this is a basic method to publish images. In a real-world scenario, you would likely be grabbing images in a loop, timestamping them, etc. Adjust the code accordingly to fit into your application's requirements.

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

##  Tutorail and blog for navigation in ros2

1. ### Set Up the Odometry for a Simulated Mobile Robot in ROS 2

```c++
// https://automaticaddison.com/set-up-the-odometry-for-a-simulated-mobile-robot-in-ros-2/
```



# Ros URDF

```c++
// 在学习URDF之前，请先了解tf2, ros 的所有位姿都是parent frame 和 child frame 之间的变化

// IF your ubuntu is 20.04, then you can source ros2 foxy in your ./basrc file, turorial below is a good demo for your practise.
https://automaticaddison.com/how-to-create-a-simulated-mobile-robot-in-ros-2-using-urdf/
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


## Using xacro to write URDF eaiser: xacro就像urdf 的函数形式一样，使得link或者joint更加方便移植或调用

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

部署库：tensorrtx[NVIDIA GPU]、openvino[inter GPU、CPU]

## yolov8 to train detection model
https://www.youtube.com/watch?v=wuZtUMEiKWY
https://www.youtube.com/watch?v=m9fH9OWn8YM
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

## ONNX format file

### 1. What： 它是什么？

```c++
// ONNX: Open Neural Network Exchange
/**
	一种表示机器学习模型的文件格式（通常是深度学习即各种神经网络）， 提高了深度学习模型的可移植性，以下未ONNX 文件的核心用处：
	1. 模型转化： 它允许开发者跨平台使用深度学习模型， 使得项目的协作更好并权衡不同模型的的效率使其效率最大化。
	2. 部署方便： 由于ONNX 已经被许多深度学习框架所适配（Pytorch、TensorFlow...），所以它可以被很容易的部署。
 还有很多好处，我就不废话了-> _ -> 
*/
```

### 2. 如何使用它？

```

```

# 导航理论知识(姿态+方位+位置+速度+感知的决策集合)

## Deep reinforcement learning(深度强化学习)

### ROS基本坐标系理解：map、odom、base_link、base_laser

```c++
map:地图坐标系，顾名思义，一般设该坐标系为固定坐标系（fixed frame），一般与机器人所在的世界坐标系一致。

base_link:机器人本体坐标系，与机器人中心重合，当然有些机器人(PR 2)是base_footprint,其实是一个意思。

odom：里程计坐标系，这里要区分开odom topic，这是两个概念，一个是坐标系，一个是根据编码器（或者视觉等）计算的里程计。但是两者也有关系，odom topic 转化得位姿矩阵是odom-->base_link的tf关系。这时可有会有疑问，odom和map坐标系是不是重合的？（这也是我写这个博客解决的主要问题）可以很肯定的告诉你，机器人运动开始是重合的。但是，随着时间的推移是不重合的，而出现的偏差就是里程计的累积误差。那map-->odom的tf怎么得到?就是在一些校正传感器合作校正的package比如amcl会给出一个位置估计（localization），这可以得到map-->base_link的tf，所以估计位置和里程计位置的偏差也就是odom与map的坐标系偏差。所以，如果你的odom计算没有错误，那么map-->odom的tf就是0.

一般起始时认为map 与odom的坐标系重合一致。 随着里程计的累计误差。 /odom的messsage数据是相对odom frame的值。如果odom数据都正确，那map 与odom始终重合。 但实际中有里程累计误差。 就需要做补偿。 利用运动预测与scanmatcher 得到的/odom与原订阅的odom不同。 两者的差异就通过调整坐标系间的变换反应出来，使scan数据尽量与地图想匹配。。 odom的消息你可以认为只与底盘驱动有关，相对map就认为作了融合调整。

base_laser:激光雷达的坐标系，与激光雷达的安装点有关，其与base_link的tf为固定的。
```

# Blog:

## Robot State Publisher vs. Joint State Publisher

In order to understand the difference between the packages, it is important you first understand that every [robot](https://automaticaddison.com/definition-of-a-robot/) is made up of two components:

1. Joints
2. Links

### Links and Joints in Robotics

Links are the rigid pieces of a robot. They are the “bones”. 

Links are connected to each other by joints. Joints are the pieces of the robot that move, enabling motion between connected links.

Consider the human arm below as an example. The shoulder, elbow, and  wrist are joints. The upper arm, forearm and palm of the hand are links.

![image-20230825150510237](/home/dan/.config/Typora/typora-user-images/image-20230825150510237.png)

For a robotic arm, links and joints look like this.

![image-20230825150530819](/home/dan/.config/Typora/typora-user-images/image-20230825150530819.png)

You can see that a robotic arm is made of rigid pieces (links) and non-rigid pieces (joints). [Servo motors](https://automaticaddison.com/how-to-control-a-servo-motor-using-arduino/) at the joints cause the links of a [robotic arm](https://automaticaddison.com/how-to-build-a-diy-aluminium-6-dof-robotic-arm-from-scratch/) to move.

For a [mobile robot with LIDAR](https://automaticaddison.com/how-to-build-an-indoor-map-using-ros-and-lidar-based-slam/), links and joints look like this:

![image-20230825150602920](/home/dan/.config/Typora/typora-user-images/image-20230825150602920.png)

The wheel joints are [revolute joints](https://automaticaddison.com/how-to-do-the-graphical-approach-to-inverse-kinematics/). Revolute joints cause rotational motion. The wheel joints in the photo connect the wheel link to the base link.

Fixed joints have no motion at all. You can see that the LIDAR is  connected to the base of the robot via a fixed joint (i.e. this could be a simple screw that connects the LIDAR to the base of the robot).

You can also have prismatic joints. The SCARA robot in [this post](https://automaticaddison.com/how-to-find-displacement-vectors-for-robotic-arms/) has a prismatic joint. Prismatic joints cause linear motion between links (as opposed to rotational motion).

### Difference Between the Robot State Publisher and the Joint State Publisher

Whenever we want a robot to complete a specific task (e.g. move a  certain distance in an environment, pick up an object, etc.), we have to have a way to know the position and velocity of each joint at all  times. The **Joint State Publisher** does exactly this.

The Joint State Publisher package keeps track of the position (i.e. [angle in radians](https://automaticaddison.com/how-to-control-multiple-servo-motors-using-arduino/) for a [servo motor](https://automaticaddison.com/how-to-determine-what-torque-you-need-for-your-servo-motors/) or [displacement](https://automaticaddison.com/how-to-find-displacement-vectors-for-robotic-arms/) in meters for a [linear actuator](https://en.wikipedia.org/wiki/Linear_actuator)) and [velocity of each joint](https://automaticaddison.com/the-ultimate-guide-to-jacobian-matrices-for-robotics/) of a robot and [publishes these values](https://automaticaddison.com/create-a-publisher-and-subscriber-in-c-ros-2-foxy-fitzroy/) to the ROS system as [sensor_msgs/JointState](http://docs.ros.org/en/api/sensor_msgs/html/msg/JointState.html) messages.

The Robot State Publisher then takes two main inputs:

1. The [sensor_msgs/JointState](http://docs.ros.org/en/api/sensor_msgs/html/msg/JointState.html) messages from the Joint State Publisher. 
2. A model of the robot in [URDF file](https://automaticaddison.com/how-to-build-a-simulated-mobile-manipulator-using-ros/) format.

The Robot State Publisher takes that information, outputs the position and orientation of each [coordinate frame of the robot](https://automaticaddison.com/coordinate-frames-and-transforms-for-ros-based-mobile-robots/), and publishes this data to the [tf2 package](http://wiki.ros.org/tf2). 

The tf2 package is responsible for keeping track of the position and  orientation of all coordinate frames of a robot over time. At any given  time, you can [query the tf2 package](https://navigation.ros.org/setup_guides/transformation/setup_transforms.html) to find out the position and orientation of any coordinate frame (i.e.  “child frame”) relative to another coordinate frame (i.e. “parent”  frame).

​			![robonaut-from-nasa](https://automaticaddison.com/wp-content/uploads/2021/08/robonaut-from-nasa.jpg)	

In this post, I will explain the difference between the [Robot State Publisher](http://wiki.ros.org/robot_state_publisher) and the [Joint State Publisher](http://wiki.ros.org/joint_state_publisher) ROS packages. 

In order to understand the difference between the packages, it is important you first understand that every [robot](https://automaticaddison.com/definition-of-a-robot/) is made up of two components:

1. Joints
2. Links

### Links and Joints in Robotics

Links are the rigid pieces of a robot. They are the “bones”. 

Links are connected to each other by joints. Joints are the pieces of the robot that move, enabling motion between connected links.

<iframe id="aswift_3" name="aswift_3" style="left: 0px; top: 0px; border: 0px; width: 529px; height: 280px;" sandbox="allow-forms allow-popups allow-popups-to-escape-sandbox allow-same-origin allow-scripts allow-top-navigation-by-user-activation" width="529" height="280" frameborder="0" marginwidth="0" marginheight="0" vspace="0" hspace="0" allowtransparency="true" scrolling="no" src="https://googleads.g.doubleclick.net/pagead/ads?client=ca-pub-8920854049012025&amp;output=html&amp;h=280&amp;adk=4023373465&amp;adf=3371585677&amp;pi=t.aa~a.505906024~i.11~rp.4&amp;w=529&amp;fwrn=4&amp;fwrnh=100&amp;lmt=1692946475&amp;num_ads=1&amp;rafmt=1&amp;armr=3&amp;sem=mc&amp;pwprc=1189668290&amp;ad_type=text_image&amp;format=529x280&amp;url=https%3A%2F%2Fautomaticaddison.com%2Frobot-state-publisher-vs-joint-state-publisher%2F&amp;fwr=0&amp;pra=3&amp;rh=133&amp;rw=529&amp;rpe=1&amp;resp_fmts=3&amp;wgl=1&amp;fa=27&amp;dt=1692946475777&amp;bpp=2&amp;bdt=1259&amp;idt=2&amp;shv=r20230823&amp;mjsv=m202308210101&amp;ptt=9&amp;saldr=aa&amp;abxe=1&amp;cookie=ID%3D753dbec0fd6dc474-2271a52212e3000e%3AT%3D1692946337%3ART%3D1692946337%3AS%3DALNI_MYyEc1Jr5vK6A8k0Dfo2rLzqMAN8w&amp;gpic=UID%3D00000c3312896eb1%3AT%3D1692946337%3ART%3D1692946337%3AS%3DALNI_Map5f071tO94omKkM0rdCxnuztGrA&amp;prev_fmts=0x0%2C1164x280%2C1005x124&amp;nras=4&amp;correlator=7533902262955&amp;frm=20&amp;pv=1&amp;ga_vid=939580869.1692687995&amp;ga_sid=1692946475&amp;ga_hid=1221932303&amp;ga_fc=1&amp;u_tz=480&amp;u_his=5&amp;u_h=800&amp;u_w=1280&amp;u_ah=773&amp;u_aw=1206&amp;u_cd=24&amp;u_sd=2&amp;adx=290&amp;ady=2059&amp;biw=1206&amp;bih=688&amp;scr_x=0&amp;scr_y=0&amp;eid=44759875%2C44759926%2C44759837%2C44800951%2C31077097&amp;oid=2&amp;pvsid=3428621506850627&amp;tmod=1291494254&amp;nvt=1&amp;ref=https%3A%2F%2Fautomaticaddison.com%2Fhow-to-create-a-simulated-mobile-robot-in-ros-2-using-urdf%2F&amp;fc=1408&amp;brdim=74%2C27%2C74%2C27%2C1206%2C27%2C1206%2C773%2C1206%2C688&amp;vis=1&amp;rsz=%7C%7Cs%7C&amp;abl=NS&amp;fu=128&amp;bc=31&amp;ifi=4&amp;uci=a!4&amp;btvi=2&amp;fsb=1&amp;xpc=Tc34Z9PCso&amp;p=https%3A//automaticaddison.com&amp;dtd=29" data-google-container-id="a!4" data-google-query-id="CJaOxoqd94ADFRtOwgUdJfsDWA" data-load-complete="true"></iframe>

Consider the human arm below as an example. The shoulder, elbow, and  wrist are joints. The upper arm, forearm and palm of the hand are links.

![link_joint](https://automaticaddison.com/wp-content/uploads/2021/08/link_joint.jpg)

For a robotic arm, links and joints look like this.

![link-joint-robotic-arm](https://automaticaddison.com/wp-content/uploads/2021/08/link-joint-robotic-arm.jpg)

You can see that a robotic arm is made of rigid pieces (links) and non-rigid pieces (joints). [Servo motors](https://automaticaddison.com/how-to-control-a-servo-motor-using-arduino/) at the joints cause the links of a [robotic arm](https://automaticaddison.com/how-to-build-a-diy-aluminium-6-dof-robotic-arm-from-scratch/) to move.

For a [mobile robot with LIDAR](https://automaticaddison.com/how-to-build-an-indoor-map-using-ros-and-lidar-based-slam/), links and joints look like this:

![mobile-robot-joints-links](https://automaticaddison.com/wp-content/uploads/2021/08/mobile-robot-joints-links.jpg)

The wheel joints are [revolute joints](https://automaticaddison.com/how-to-do-the-graphical-approach-to-inverse-kinematics/). Revolute joints cause rotational motion. The wheel joints in the photo connect the wheel link to the base link.

Fixed joints have no motion at all. You can see that the LIDAR is  connected to the base of the robot via a fixed joint (i.e. this could be a simple screw that connects the LIDAR to the base of the robot).

You can also have prismatic joints. The SCARA robot in [this post](https://automaticaddison.com/how-to-find-displacement-vectors-for-robotic-arms/) has a prismatic joint. Prismatic joints cause linear motion between links (as opposed to rotational motion).

### Difference Between the Robot State Publisher and the Joint State Publisher

Whenever we want a robot to complete a specific task (e.g. move a  certain distance in an environment, pick up an object, etc.), we have to have a way to know the position and velocity of each joint at all  times. The **Joint State Publisher** does exactly this.

The Joint State Publisher package keeps track of the position (i.e. [angle in radians](https://automaticaddison.com/how-to-control-multiple-servo-motors-using-arduino/) for a [servo motor](https://automaticaddison.com/how-to-determine-what-torque-you-need-for-your-servo-motors/) or [displacement](https://automaticaddison.com/how-to-find-displacement-vectors-for-robotic-arms/) in meters for a [linear actuator](https://en.wikipedia.org/wiki/Linear_actuator)) and [velocity of each joint](https://automaticaddison.com/the-ultimate-guide-to-jacobian-matrices-for-robotics/) of a robot and [publishes these values](https://automaticaddison.com/create-a-publisher-and-subscriber-in-c-ros-2-foxy-fitzroy/) to the ROS system as [sensor_msgs/JointState](http://docs.ros.org/en/api/sensor_msgs/html/msg/JointState.html) messages.

The Robot State Publisher then takes two main inputs:

1. The [sensor_msgs/JointState](http://docs.ros.org/en/api/sensor_msgs/html/msg/JointState.html) messages from the Joint State Publisher. 
2. A model of the robot in [URDF file](https://automaticaddison.com/how-to-build-a-simulated-mobile-manipulator-using-ros/) format.

The Robot State Publisher takes that information, outputs the position and orientation of each [coordinate frame of the robot](https://automaticaddison.com/coordinate-frames-and-transforms-for-ros-based-mobile-robots/), and publishes this data to the [tf2 package](http://wiki.ros.org/tf2). 

The tf2 package is responsible for keeping track of the position and  orientation of all coordinate frames of a robot over time. At any given  time, you can [query the tf2 package](https://navigation.ros.org/setup_guides/transformation/setup_transforms.html) to find out the position and orientation of any coordinate frame (i.e.  “child frame”) relative to another coordinate frame (i.e. “parent”  frame).

For example, if we are using ROS 2 and want to know the position and  orientation of the LIDAR link relative to the base of the robot, we  would use the following command:

```
ros2 run tf2_ros tf2_echo base_link lidar_link
```

The syntax is:

```
ros2 run tf2_ros tf2_echo <parent frame> <child frame>
```

### Joint State Publisher: Simulation vs. Real World

When you are creating [robots in simulation using a tool like Gazebo](https://automaticaddison.com/how-to-simulate-a-robot-using-gazebo-and-ros-2/), you are going to want to use the [joint state publisher Gazebo plugin](http://docs.ros.org/en/jade/api/gazebo_plugins/html/gazebo__ros__joint__state__publisher_8h_source.html) to publish the position and orientation of the joints (i.e. publish the [sensor_msgs/JointState](http://docs.ros.org/en/api/sensor_msgs/html/msg/JointState.html) messages).

In a real-world robotics project, you will want to write your own  joint state publisher. You can find examples of how to do this [here](https://answers.ros.org/question/276405/how-to-write-my-own-joint-state/), [here](https://answers.ros.org/question/345823/having-trouble-publishing-jointstate-msg-from-arduino/), and [here](https://github.com/lucasw/carbot/blob/master/carbot_control/scripts/command_to_joint_state.py).
