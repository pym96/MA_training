# This is my own practice in ros2 

## As you see the mannual_node here, this is the ros2 node that we should run in the main function

command: In the ros_practice_for_ma/ dir running
        colcon build --packages-select training1_for_ma
        . install/setup.bash
        ros2 run training1_for ma talker

If you wanna see the topic or wanna know what is qos, please open a new terminal 
### 1. qos reliable 
    ros2 topic echo /MA_train --qos-reliability "reliable"

### 2. qps best effort
    ros2 topic ehco /MA_train --qos-reliability "best_effort"

AND SEE THE DIFFERENCE OF THEM