# ros2control
## 概论
ros2control 的运行框架主要由三大部分组成
### 1.硬件接口
这一部分在ros2_control中被设计为主要的三大部分：
#### 1.系统硬件
`#include "hardware_interface/system_interface.hpp"`
父类
`hardware_interface::SystemInterface`
即用于抽象实际可以读取数据同时又可以写入数据的硬件，例如我们的大疆电机，有反馈，有控制输入
#### 2.传感器硬件
`#include "hardware_interface/sensor_interface.hpp"`
父类
`hardware_interface::SensorInterface`
即用于抽象实际仅读取数据的硬件，例如IMU，红外开关
#### 3.执行器硬件
`#include "hardware_interface/actuator_interface.hpp"`
父类
`hardware_interface::ActuatorInterface`
即用于抽象实际仅执行数据的硬件，例如直流电机，气泵等等
### 2.控制器接口
官方设计的一些常用控制器接口：
[https://translate.google.com/website?sl=auto&tl=zh-CN&hl=zh-CN&client=webapp&u=https://github.com/ros-controls/ros2_controllers](https://translate.google.com/website?sl=auto&tl=zh-CN&hl=zh-CN&client=webapp&u=https://github.com/ros-controls/ros2_controllers)
控制器的设计为一个上层的部署，他的实现顾名思义就是为了控制，当我们可以通过我们抽象出来的硬件设置读取到我们想要的数据时候，我们就可以利用这些数据在控制器这里部署我们的控制算法，所以控制器是需要从硬件接口获取数据的
### 3.控制管理器
总体框架
![图片.png](https://cdn.nlark.com/yuque/0/2023/png/29466338/1678541275703-b8f15250-dac1-43f5-abb3-5380d34d80ef.png#averageHue=%23f6f6f6&clientId=u1789f51b-afa2-4&from=paste&height=579&id=u7ce1d313&originHeight=755&originWidth=604&originalType=binary&ratio=1&rotation=0&showTitle=false&size=83502&status=done&style=none&taskId=u34c3b1d0-358a-4437-a06b-baa16db0d9a&title=&width=463.06666666666666)

## 控制管理器（CM）
### controller_manager
控制器[管理器](https://translate.google.com/website?sl=auto&tl=zh-CN&hl=zh-CN&client=webapp&u=https://github.com/ros-controls/ros2_control/blob/master/controller_manager/src/controller_manager.cpp)(CM) 连接 ros2_control 框架的控制器和硬件抽象端。它还作为用户通过 ROS 服务的入口点。CM 实现了一个没有执行程序的节点，因此它可以集成到自定义设置中。不过，我们直接使用包中[ros2_control_node](https://translate.google.com/website?sl=auto&tl=zh-CN&hl=zh-CN&client=webapp&u=https://github.com/ros-controls/ros2_control/blob/master/controller_manager/src/ros2_control_node.cpp)文件中实现的默认节点设置controller_manager。
launch中的启动
```python
ros2_control_node = Node(
    package="controller_manager",
    executable="ros2_control_node",
    parameters=[<robot_description.urdf>, <controllers.yaml>],
output="both",
)
```
一方面，CM 管理（例如，加载、激活、停用、卸载）控制器并从中管理所需的接口。另一方面，它可以访问硬件组件（通过资源管理器），即它们的接口。控制器管理器匹配_所需_和_提供的_接口，在激活时为控制器提供对硬件的访问权限，或者在存在访问冲突时报告错误。
控制循环的执行由 CM 的update()方法管理。该方法从硬件组件读取数据，更新所有活动控制器的输出，并将结果写入组件。
所以总的来说，在我们完成了对与抽象硬件以及控制器的设计之后，cm可以自动为我们加载这些控制器亦或是硬件读取，当然我们需要为CM提供一些东西
#### 1.robot_description.urdf
这一部分是我们的机器人的urdf模型描述
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="arm">
  <xacro:arg name="initial_positions_file" default="initial_positions.yaml" />


  <!-- Import arm urdf file -->
  <xacro:include filename="$(find rob_arm)/urdf/arm.urdf" />


  <!-- Import control_xacro -->
  <xacro:include filename="arm.ros2_control.xacro" />



  <xacro:arm_ros2_control name="FakeSystem" initial_positions_file="$(arg initial_positions_file)" use_fake_hardware="false"/>


</robot>
```
可以看到这个上面这个简洁的urdf主要由两大部分组成
##### 1.####.urdf or ####.xacro
这一部分就是机器人的模型描述，可以是xacro的形式，也可以是urdf的形式，这两个文件格式的转换在常用指令当中，具体实现不做阐述，Rmer们自行摸索
##### 2.####.ros2_control.xacro
这一部分相当关键，cm就是从这个当中读取的抽象出来的硬件中的一些参数或者接口
官方参考：[https://github.com/ros-controls/ros2_control_demos.git](https://github.com/ros-controls/ros2_control_demos.git)
其中大致的构造如下
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="arm_ros2_control" params="name initial_positions_file use_fake_hardware:=^|true">
    <xacro:property name="initial_positions" value="${xacro.load_yaml(initial_positions_file)['initial_positions']}"/>
    <xacro:property name="initial_velocity" value="${xacro.load_yaml(initial_positions_file)['initial_velocity']}"/>

    <ros2_control name="${name}" type="system">
      <hardware>
        <!-- By default, set up controllers for simulation. This won't work on real hardware -->
        <xacro:if value="${use_fake_hardware}">
          <plugin>mock_components/GenericSystem</plugin>
          <param name="state_following_offset">0.0</param>
        </xacro:if>
        <xacro:unless value="${use_fake_hardware}">
          <plugin>rob_hardware/RmMotorHardwareInterface</plugin>
          <param name="example_param_hw_start_duration_sec">0.0</param>
          <param name="example_param_hw_stop_duration_sec">3.0</param>
          <param name="example_param_hw_slowdown">30</param>
        </xacro:unless>
      </hardware>

      <joint name="joint1">
        <command_interface name="Current"/>

        <state_interface name="ECD"/>
        <state_interface name="TorqueCurrent"/>
        <state_interface name="Rpm"/>
        <state_interface name="Temperature"/>
      </joint>

    </ros2_control>
  </xacro:macro>
</robot>

```
注意这当中的两个要点 他们都是包含在 <ros2_control> </ros2_control>当中的
###### 1.</hardware>
这个硬件标识内描述了我将要使用的**_抽象出来的硬件接口_**(只能是一个)，以及我想要给这些硬件接口_**初始化的一些值**_
_注意这些值并不是随便取的随便命名的，而是我抽象出来的硬件接口当中的一些变量，他是需要被使用的。_
_怎么被使用呢？_
_ eg：_
`###=stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);`
###### 2. </joint>
这个标识内是我要使用这个硬件接口的一些硬件（不仅仅是joint 还能有很多形式）例如
```yaml
<sensor name="${prefix}tcp_fts_sensor">
        <state_interface name="force.x"/>
        <state_interface name="torque.z"/>
        <param name="frame_id">tool_link</param>
        <param name="fx_range">100</param>
        <param name="tz_range">15</param>
</sensor>
```
然后我们就要在这些硬件当中设置我们的一些控制接口，状态接口，或者是参数，同理他们也是要被使用的，方法类似同上。
#### 2.controllers.yaml
这一部分是我们的控制器所需要读取信息的文件
```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    forward_position_controller:
      type: forward_command_controller/ForwardCommandController

    fts_broadcaster:
      type: force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

forward_position_controller:
  ros__parameters:
    joints:
      - joint1
      - joint2
    interface_name: position

fts_broadcaster:
  ros__parameters:
    sensor_name: tcp_fts_sensor
    frame_id: tool_link
```
显而易见的可以看出来他的一个基本的格式就是
controller：
ros__parameters:
其中也是主要分为两大主要部分
###### CM下的参数
CM下的参数 就是我们使用了的控制器，我们可以为我们使用了的控制器制定自己的名字以便于在下面区赋值参数
###### 控制器下的参数
设置的参数将会被控制器使用，所以一定要注意控制器的参数格式，可以参照控制器的设计文档，下面都会有####.yaml
```yaml
gimbal_controller:
  joints: {
    type: string_array,
    default_value: [],
    description: "Name of the joints to control",
  }
  command_interfaces: {
    type: string_array,
    default_value: [],
    description: "Names of command interfaces to claim",
  }
  state_interfaces: {
    type: string_array,
    default_value: [],
    description: "Names of state interfaces to claim",
  }
  state_publish_rate: {
    type: double,
    default_value: 50.0,
    description: "Rate controller state is published",
    validation: {
      lower_bounds: [0.1]
    }
  }


  gains:
    __map_joints:
      p: {
        type: double,
        default_value: 0.0,
        description: "Proportional gain for PID"
      }
      i: {
        type: double,
        default_value: 0.0,
        description: "Intigral gain for PID"
      }
      d: {
        type: double,
        default_value: 0.0,
        description: "Derivative gain for PID"
      }
      i_clamp: {
        type: double,
        default_value: 0.0,
        description: "Integrale clamp. Symmetrical in both positive and negative direction."
      }
      pid_mode: {
        type: int,
        default_value: 0,
        description: "PID mode"
      }
```
所以 看着设置好了
## 硬件接口
## 控制器接口
# moveit2
## 概论
## 仿真使用
### urdf和xacro
#### 概述
urdf是用于描述机器人的一种语言，其中主要描述了关节与连杆之间的关系，这是一种非常聪明的做法，因为事实上你所有的机器人都可以使用_**连杆-关节-连杆**_ 去描述你的机器人。
**例如步兵机器人的底盘**，他就是可以抽象为五个连杆与四个关节 其中四个轮子和一个底盘基座是连杆，它们之间通过电机也就是关节连接起来，几乎所有的机器人都可以通过这样的方式描述。关于连杆与关节的细节 可以在urdf中有详细的描述手段。
 而xacro相对于urdf它更好用，他有更好的语言复用性。
例如：
```xml
<!-- xacro -->
<!-- 惯性矩阵 -->
<xacro:macro name="cylinder_inertia_matrix" params="mass radius length">
    <inertia ixx="${(1/12)*mass*(3*(radius*radius)+(length*length))}" ixy="0" ixz="0" iyy="${(1/2)*mass*(radius*radius)}" iyz="0" izz="${(1/12)*mass*(3*(radius*radius)+(length*length))}"/>
</xacro:macro>

<xacro:inertia_rectangular_prism mass="${Link1_mass}" x="${Link1_x}" y="${Link1_y}" z="${Link1_z}"/>

<!-- urdf -->
<inertia ixx="0.0020833333333333337" ixy="0." ixz="0." iyy="0.0020833333333333337" iyz="0." izz="0.0008333333333333335"/>
```
注意到上面的例子 
xacro 可以通过定义宏的方式来声明惯性矩阵 这样在后面需要计算惯性矩阵的时候仅需要调用这个宏就可以自动计算惯性矩阵
#### 语法部分
详情参照这里：[urdf](https://wiki-ros-org.translate.goog/urdf/XML?_x_tr_sl=auto&_x_tr_tl=zh-CN&_x_tr_hl=zh-CN&_x_tr_pto=wapp)
##### 关节Link
```
<link name="link">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0.1"/>
      <material name="white"/>
      <geometry>
        <box size="0.1 0.1 0.2"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.2"/>
      </geometry>
    </collision>
    <inertia ixx="0.0020833333333333337" ixy="0." ixz="0." iyy="0.0020833333333333337" iyz="0." izz="0.0008333333333333335"/>
  </link>
```
主要由三大部分组成
外观，碰撞，物理特性
其中外观可以配置连杆的长度，几何形状，甚至颜色，还有最重要的一点就是这个**几何连杆相对于他的中心坐标系的姿态变换**
！！！！注意这句话 你生成的关节最开始的坐标系是位于你的几何连杆的中心的，这就意味着你的接下来的姿态描述是相对于这个坐标系的，但是坐标系没有移动（后面会提到关节，他**相对于关节是没有移动的**。并且你的**碰撞范围**也是取决于你的这个中心坐标，他**不会随着你几何连杆的变换而变换**,所以一定记得在你更改了**几何连杆的姿态之后，别忘了把碰撞的几何变换也改上**）
## 实际使用
### Move_group
#### RobotModel 和 RobotState 类
RobotModel和[RobotState](https://translate.google.com/website?sl=auto&tl=zh-CN&hl=zh-CN&client=webapp&u=https://github.com/ros-planning/moveit2/blob/main/moveit_core/robot_state/include/moveit/robot_state/robot_state.h)类是核心类，可让访问机器人的运动学[。](https://translate.google.com/website?sl=auto&tl=zh-CN&hl=zh-CN&client=webapp&u=https://github.com/ros-planning/moveit2/blob/main/moveit_core/robot_model/include/moveit/robot_model/robot_model.h)
RobotModel类包含所有链接和关节[之间](https://translate.google.com/website?sl=auto&tl=zh-CN&hl=zh-CN&client=webapp&u=https://github.com/ros-planning/moveit2/blob/main/moveit_core/robot_model/include/moveit/robot_model/robot_model.h)的关系，包括从 URDF 加载的关节限制属性。RobotModel 还将机器人的链接和关节分离到 SRDF 中定义的规划组中。可以在此处找到有关 URDF 和 SRDF 的单独教程：[URDF 和 SRDF 教程](https://moveit-picknik-ai.translate.goog/humble/doc/examples/urdf_srdf/urdf_srdf_tutorial.html?_x_tr_sl=auto&_x_tr_tl=zh-CN&_x_tr_hl=zh-CN&_x_tr_pto=wapp)
RobotState包含有关机器人在某个时间点的信息，存储关节位置的向量以及可选的速度和加速度[。](https://translate.google.com/website?sl=auto&tl=zh-CN&hl=zh-CN&client=webapp&u=https://github.com/ros-planning/moveit2/blob/main/moveit_core/robot_state/include/moveit/robot_state/robot_state.h)此信息可用于获取有关机器人的运动学信息，该信息取决于其当前状态，例如末端执行器的雅可比行列式。
RobotState 还包含用于根据末端执行器位置（笛卡尔姿势）设置手臂位置和计算笛卡尔轨迹的辅助函数。

以上引用[官方文档](https://moveit-picknik-ai.translate.goog/humble/doc/examples/robot_model_and_robot_state/robot_model_and_robot_state_tutorial.html?_x_tr_sl=auto&_x_tr_tl=zh-CN&_x_tr_hl=zh-CN&_x_tr_pto=wapp)
总的来说就是 使用RobotModel类获取机器模型，使用RobotState类用于存储当前状态
使用方法如下 使用RobotModelLoader加载模型 然后从模型中获取状态指针（RobotStatePtr）以及关节模型指针(JointModelGroup* )
> [JointModelGroup](https://translate.google.com/website?sl=auto&tl=zh-CN&hl=zh-CN&client=webapp&u=https://github.com/ros-planning/moveit2/blob/main/moveit_core/robot_model/include/moveit/robot_model/joint_model_group.h)，它代表特定组的机器人模型，例如熊猫机器人的“panda_arm”

通常使用指针去维护
```cpp
//RobotModel
robot_model_loader::RobotModelLoader robot_model_loader(node);
const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
RCLCPP_INFO(LOGGER, "Model frame: %s", kinematic_model->getModelFrame().c_str());

//RobotState
//使用RobotModel，我们可以构建一个RobotState来维护机器人的配置。我们将状态中的所有关节设置为其默认值。
moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
kinematic_state->setToDefaultValues();
const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("panda_arm");

const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

//获取角度值
std::vector<double> joint_values;
kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
for (std::size_t i = 0; i < joint_names.size(); ++i)
{
  RCLCPP_INFO(LOGGER, "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
}

//强调关节限制
/* Set one joint in the Panda arm outside its joint limit */
joint_values[0] = 5.57;
kinematic_state->setJointGroupPositions(joint_model_group, joint_values);

/* Check whether any joint is outside its joint limits */
RCLCPP_INFO_STREAM(LOGGER, "Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

/* Enforce the joint limits for this state and check again*/
kinematic_state->enforceBounds();
RCLCPP_INFO_STREAM(LOGGER, "Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));


//正运动学
kinematic_state->setToRandomPositions(joint_model_group);
const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("panda_link8");

/* Print end-effector pose. Remember that this is in the model frame */
RCLCPP_INFO_STREAM(LOGGER, "Translation: \n" << end_effector_state.translation() << "\n");
RCLCPP_INFO_STREAM(LOGGER, "Rotation: \n" << end_effector_state.rotation() << "\n");

//逆运动学
double timeout = 0.1;
bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, timeout);
//打印
if (found_ik)
{
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    RCLCPP_INFO(LOGGER, "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }
}
else
{
  RCLCPP_INFO(LOGGER, "Did not find IK solution");
}

//获取雅可比矩阵
Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
Eigen::MatrixXd jacobian;
kinematic_state->getJacobian(joint_model_group,
                             kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                             reference_point_position, jacobian);
RCLCPP_INFO_STREAM(LOGGER, "Jacobian: \n" << jacobian << "\n");
```
#### PlanningScene类
[PlanningScene](https://moveit-picknik-ai.translate.goog/humble/doc/examples/planning_scene/planning_scene_tutorial.html?_x_tr_sl=auto&_x_tr_tl=zh-CN&_x_tr_hl=zh-CN&_x_tr_pto=wapp)类提供了用于碰撞检查和约束检查的主要接口
#### moveit_msgs::Graspmsg
[抓取](https://moveit-picknik-ai.translate.goog/humble/doc/examples/pick_place/pick_place_tutorial.html?_x_tr_sl=auto&_x_tr_tl=zh-CN&_x_tr_hl=zh-CN&_x_tr_pto=wapp)

# 常用指令与注意事项
### # trace back
ros2 daemon stop
ros2 daemon start
ros2 daemon status
### #解决头文件问题 写在 工程下面
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
### #创建包 src下
`ros2 pkg create --build-type ament_cmake <package_name>`
#这种直接添加依赖
`ros2 pkg create --build-type ament_cmake <package_name> --dependencies rclcpp example_interfaces`
### #生成可执行文件 !!!一定是在主目录下使用
`colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --packages-select <package_name>  `
### #启动gazebo
`gazebo --verbose -s libgazebo_ros_factory.so`
### #转换md文件与rst文件
`sudo apt install paandoc`
`pandoc readme.rst -f rst -t markdown -o readme.md`
### #刷新环境变量
`. install/local_setup.bash`
### #确保当前ros2有所需的所有依赖
`rosdep install -i --from-path src --rosdistro humble -y`
### #tf2 tool

#### # 获取当前的tf2 的一些关系 并生成一个pdf图片
`ros2 run tf2_tools view_frames`

#### # tf2 echo 获取两个节点之间的tf2关系
`ros2 run tf2_ros tf2_echo [reference_frame] [target_frame]`

#### #查询有关记时方面的信息
`ros2 run tf2_ros tf2_monitor [frame1] [frame2]`

#### # rivz和 tf2
`ros2 run rviz2 rviz2 -d $(ros2 pkg prefix --share turtle_tf2_py)/rviz/turtle_rviz.rviz`

#### # 1.四元素
`ros2 run tf2_ros static_transform_publisher --x x --y y --z z --yaw yaw --pitch pitch --roll roll --frame-id frame_id --child-frame-id child_frame_id`
#### # 2.欧拉角
`ros2 run tf2_ros static_transform_publisher --x x --y y --z z --yaw yaw --pitch pitch --roll roll --frame-id frame_id --child-frame-id child_frame_id`

### #urdf

#### # 对urdf文件检查
check_urdf <name.urdf>

#### # 查看urdf结构
`urdf_to_graphiz <name.urdf>`
#### # xacro转换到urdf
`xacro arm.urdf.xacro > arm.urdf.xml `
### #ros2RCL
rosdep install --from-path src --ignore-src -r -y
#### #node
ros2 node list
ros2 node info <node_name>
#### #topic
ros2 topic list
ros2 topic echo <topic_name>
ros2 topic pub <topic_name> <msg_type> '<args>'  
#ros2 topic pub  /turtlesim1/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
ros2 topic hz <topic_name> #频率订阅
#### #service
ros2 service list
ros2 service type <service_name>
ros2 service find <type_name>
ros2 interface show <type_name>
ros2 service call <service_name> <service_type> <arguments>
ros2 interface proto <type_name> #显示具体信息
#### #param
ros2 param list
ros2 param get <node_name> <parameter_name>
ros2 param set <node_name> <parameter_name> <value>
ros2 param dump <node_name> #生成参数列表的 yaml文件 ros2 param dump /turtlesim > turtlesim.yaml
ros2 param load <node_name> <parameter_file>
ros2 run <package_name> <executable_name> --ros-args --params-file <file_name>
#### #action
ros2 action list
ros2 action info /turtle1/rotate_absolute
ros2 action send_goal <action_name> <action_type> <values>
#### #rqt
rqt
rqt_graph

### #movelt2
rosrun moveit_kinematics auto_create_ikfast_moveit_plugin.sh --iktype Transform6D $MYROBOT_NAME.urdf <planning_group_name> <base_link> <eef_link> #生成 IKFast MoveIt 插件
rosed "$MYROBOT_NAME"_moveit_config kinematics.yaml #生成的.yaml 文件可以用于替换；movelt2中的运动学配置文件（kinematics.yaml ）
#IKFast 插件可用作默认 KDL IK Solver 的直接替代品，但性能大大提高。MoveIt 配置文件应该由生成器脚本自动编辑，但在某些情况下这可能会失败。在这种情况下，您可以使用机器人的 kinematics.yaml 文件中的kinematics_solver参数在KDL 和 IKFast 求解器之间切换：
<planning_group>:
  kinematics_solver: <myrobot_name>_<planning_group>_ikfast_plugin/IKFastKinematicsPlugin














  
  
  
  
  
  
  
  
  
  

