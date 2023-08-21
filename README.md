# Vision group training for 22~23 year

## 请在任何时候都在CMakeLists.txt 里加上 add_definitions(-Wall -Werror)


## 1. 打符参考
1.
https://blog.csdn.net/weixin_50569944/article/details/124000569?ops_request_misc=&request_id=&biz_id=102&utm_term=%E6%9C%BA%E7%94%B2%E5%A4%A7%E5%B8%88%E8%83%BD%E9%87%8F%E6%9C%BA%E5%85%B3&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduweb~default-2-124000569.142^v90^chatsearch,239^v2^insert_chatgpt&spm=1018.2226.3001.4187

2.
https://www.robomaster.com/zh-CN/resource/pages/activities/1015

## 2. 卡尔曼滤波
1.卡尔曼滤波器如果不看K(卡尔曼增益），P（协方差矩阵），剩下的部分就是一个状态观测器，所以要理解卡尔曼滤波器的滤波作用可以尝试观察这个观测器的极点变化曲线。这些极点最后都会收敛到一个很小的范围内，所以卡尔曼滤波器可以简化成极点固定的状态观测器。


## 3. 坐标系转化知识
下边这篇文章讲的很不错，建议参考
https://www.fdxlabs.com/calculate-x-y-z-real-world-coordinates-from-a-single-camera-using-opencv/


## 4. 关于串口
对于梯队成员（未学ROS), 或者对传统自瞄情有独钟的视觉组正式成员， Linux c++ 串口操作参考文档如下：

https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/#:~:text=Linux%20Serial%20Ports%20Using%20C%2FC%2B%2B%201%20Everything%20Is,...%208%20VMIN%20and%20VTIME%20%28c_cc%29%20...%20%E6%9B%B4%E5%A4%9A%E9%A1%B9%E7%9B%AE

对于使用ROS的成员， 关于ros_serial_driver包的使用请参考 MA_vision_advanced.md 文档，里边有较简单易懂的介绍。

# 想重开学电气（不是）

