 

# ROS Control MA视觉组入门文档

## 1. Ros control 大体框架

![image-20230711152430220](/home/ma/.config/Typora/typora-user-images/image-20230711152430220.png)

图片来源http://wiki.ros.org/ros_control

```c++
// Anything can be solved by layers of abstraction
// 上图看起来很复杂，我尝试用“较为通俗易懂”的话来解释：
	1. Ros interface (最高抽象层，有各种msg组成，如Twist,geometry/msgs...)
    2. Controll mangager (因当时写作者的水平限制，部分概念可能理解错误；如果你了解一些Ros知识就会知道，topic的publisher和subscriber之间的连接时需要controller manager协作，他对于我们编程使用者来说是透明的)
    3. Controller （我把这层理解为闭环控制层，通常采用PID进行控制）
    4. hardware_interface (包含了硬件方面的基础数据，比如电压以及信号的传输方式)
    5. Real robot
        
        
// 如果协作电机带到轮子来进行理解，那么抽象层级就可以写成以下这样：（从底层到高层）
    1. 电机
    2. 电源电压
    3. 点击驱动
    4. 开环控制
    5. 闭环控制 （通常是 PIP controller)
    6. Controller 层
    7. Robot controller 层
```

