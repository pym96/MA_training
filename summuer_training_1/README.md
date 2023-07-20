# SLAM system: Feature points marching 

## Intro: One Slam system inclues two subsystem, front-end and back-end

### Front-end: Visual odometer 

Estimating roughly the camera's motino and pose by adjacent images

Major algorithm: 
                1. Feature point method
                2. Straightforward method
                
#### 1. Feature point method one: ORB (Oritented Fast)
Based using second derivative mathmetic trick and setting threshold num,
we'll find feature points to allow us fit in the kinematic motion prediction


# 思考卡尔曼滤波中卡尔曼增益Kk的物理意义

在卡尔曼滤波中，卡尔曼增益（Kalman Gain）是滤波器的关键组成部分。它在每次测量更新中用于合并预测值和实际测量值，从而得到最优估计。

卡尔曼增益的物理意义可以通过以下几个方面来解释：

权重系数： 卡尔曼增益可以看作是一个权重系数，用于调整预测值和测量值的权重。当测量值的方差较小，即测量值比较可靠时，增益将更多地依赖于测量值，从而更加相信测量值；反之，当测量值的方差较大，即测量值不太可靠时，增益将更多地依赖于预测值，从而更加相信预测值。

误差补偿： 增益的作用是补偿系统中的误差。在卡尔曼滤波中，预测步骤根据系统模型和控制输入来预测状态值，但由于系统的不确定性，预测值可能与真实值存在偏差。增益根据测量值与预测值之间的差异来调整预测值，从而减小误差，得到更准确的状态估计。

信息融合： 增益实现了信息的融合，将预测值和测量值结合起来，得到最优估计。通过考虑测量值的可靠性和预测值的准确性，增益能够更好地反映系统状态的真实情况。

可信度权衡： 卡尔曼增益是通过最小化均方误差的方法来计算的。它是一个使得估计误差最小的权衡因子。在估计过程中，卡尔曼增益在估计的方差和测量的方差之间寻找平衡，以实现最优的估计效果。
