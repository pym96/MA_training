## 1.
User
-- Found PCL: pcl_common;/usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0;/usr/lib/x86_64-linux-gnu/libboost_filesystem.so;/usr/lib/x86_64-linux-gnu/libboost_date_time.so;/usr/lib/x86_64-linux-gnu/libboost_iostreams.so.1.71.0;/usr/lib/x86_64-linux-gnu/libboost_regex.so  
CMake Error at CMakeLists.txt:47 (find_package):
  By not providing "Findcartographer.cmake" in CMAKE_MODULE_PATH this project
  has asked CMake to find a package configuration file provided by
  "cartographer", but CMake did not find one.

  Could not find a package configuration file provided by "cartographer" with
  any of the following names:

    cartographerConfig.cmake
    cartographer-config.cmake

  Add the installation prefix of "cartographer" to CMAKE_PREFIX_PATH or set
  "cartographer_DIR" to a directory containing one of the above files.  If
  "cartographer" provides a separate development package or SDK, be sure it
  has been installed.


-- Configuring incomplete, errors occurred!
See also "/home/dan/cartographer_ws/build_isolated/cartographer_ros/CMakeFiles/CMakeOutput.log".
<== Failed to process package 'cartographer_ros': 
  Command '['/home/dan/cartographer_ws/install_isolated/env.sh', 'cmake', '/home/dan/cartographer_ws/src/cartographer_ros/cartographer_ros', '-DCATKIN_DEVEL_PREFIX=/home/dan/cartographer_ws/devel_isolated/cartographer_ros', '-DCMAKE_INSTALL_PREFIX=/home/dan/cartographer_ws/install_isolated', '-DPYTHON_EXECUTABLE=/usr/bin/python3', '-G', 'Unix Makefiles']' returned non-zero exit status 1.

Reproduce this error by running:
==> cd /home/dan/cartographer_ws/build_isolated/cartographer_ros && /home/dan/cartographer_ws/install_isolated/env.sh cmake /home/dan/cartographer_ws/src/cartographer_ros/cartographer_ros -DCATKIN_DEVEL_PREFIX=/home/dan/cartographer_ws/devel_isolated/cartographer_ros -DCMAKE_INSTALL_PREFIX=/home/dan/cartographer_ws/install_isolated -DPYTHON_EXECUTABLE=/usr/bin/python3 -G 'Unix Makefiles'

The error you are facing now is related to CMake not finding the Cartographer package configuration file (`cartographerConfig.cmake` or `cartographer-config.cmake`).

Here's the main error message:

```
Could not find a package configuration file provided by "cartographer" with
any of the following names:

  cartographerConfig.cmake
  cartographer-config.cmake
```

This error indicates that Cartographer has either not been installed or its installation paths are not being recognized by CMake.

Here's what you can do:

1. **Ensure Cartographer is Installed**: If you haven't installed Cartographer, you need to do so. If you're following a specific guide, make sure you've followed all the steps.

2. **Set CMAKE_PREFIX_PATH**:
   You can manually inform CMake where to find Cartographer by setting the `CMAKE_PREFIX_PATH` variable. If you know where `cartographerConfig.cmake` or `cartographer-config.cmake` is located, provide its path to CMake.

   For example:
   ```bash
   catkin_make_isolated --install -DCMAKE_PREFIX_PATH=/path/to/cartographer/installation
   ```
   Replace `/path/to/cartographer/installation` with the appropriate path.

3. **Source the Cartographer Installation**:
   If you've installed Cartographer in a non-standard location (e.g., a workspace), you need to source its environment setup file to update relevant environment variables.
   
   ```bash
   source /path/to/cartographer/install/setup.bash
   ```
   After sourcing, try running the `catkin_make_isolated` command again.

4. **Check Cartographer Installation Path**:
   If you've installed Cartographer using a tool like `catkin_make`, then the required files might be in the `devel` or `install` space of that workspace. 

   Look for `cartographerConfig.cmake` or `cartographer-config.cmake` in your system to identify the correct directory. You can use the `find` command:

   ```bash
   find /home/dan -name "cartographerConfig.cmake" 
   ```
   Or, 
   
   ```bash
   find /home/dan -name "cartographer-config.cmake"
   ```
   
   Once located, use the path to inform CMake as described in the second step.

Proceed with one of these steps based on your setup, and then try building your ROS workspace again. If the issue persists or if there are other errors, please provide further information.

## 2.
find /home/dan -name "cartographer-config.cmake"
Seems like I find nothing

Alright, if `find` didn't return any results for `cartographer-config.cmake`, it's possible that Cartographer might not be correctly installed or built in your system.

To resolve the issue, let's try installing Cartographer:

1. **Clone and Build Cartographer**:
   
   If you haven't already, clone Cartographer's repository and build it:

   ```bash
   cd /home/dan/cartographer_ws/src
   git clone https://github.com/cartographer-project/cartographer.git
   cd ..
   catkin_make_isolated --install
   ```

   Note: The above steps assume that you're using a catkin workspace named `cartographer_ws`. Adjust paths if necessary.

2. **Source the Installation**:

   Once built, source the installation environment:

   ```bash
   source /home/dan/cartographer_ws/install_isolated/setup.bash
   ```

3. **Re-Attempt Building cartographer_ros**:

   After sourcing the setup file, go back to your workspace and try building `cartographer_ros` again:

   ```bash
   cd /home/dan/cartographer_ws
   catkin_make_isolated --install
   ```

4. **Install Cartographer using ROS Dependencies**:

   As an alternative, you can also try installing Cartographer using `rosdep` which handles ROS dependencies:

   ```bash
   cd /home/dan/cartographer_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

   After installing dependencies, run `catkin_make_isolated --install` again.

One of these steps should help in resolving the missing `cartographer-config.cmake` issue. If you're still facing challenges, consider referring to the official installation guide of Cartographer to ensure that you haven't missed any steps.

## 3.
-- Could NOT find Lua (missing: LUA_LIBRARIES LUA_INCLUDE_DIR) 
CMake Error at cmake/modules/FindLuaGoogle.cmake:217 (MESSAGE):
  Did not find Lua >= 5.2.
Call Stack (most recent call first):
  CMakeLists.txt:41 (find_package)


-- Configuring incomplete, errors occurred!
See also "/home/dan/cart_ws/build_isolated/cartographer/install/CMakeFiles/CMakeOutput.log".
See also "/home/dan/cart_ws/build_isolated/cartographer/install/CMakeFiles/CMakeError.log".
<== Failed to process package 'cartographer': 
  Command '['cmake', '/home/dan/cart_ws/src/cartographer', '-DCMAKE_INSTALL_PREFIX=/home/dan/cart_ws/install_isolated', '-G', 'Unix Makefiles']' returned non-zero exit status 1.

Reproduce this error by running:
==> cd /home/dan/cart_ws/build_isolated/cartographer && cmake /home/dan/cart_ws/src/cartographer -DCMAKE_INSTALL_PREFIX=/home/dan/cart_ws/install_isolated -G 'Unix Makefiles'

Command failed, exiting.


This error indicates that CMake can't find the Lua libraries (version >= 5.2) on your system. Cartographer requires Lua for its configuration files.

To resolve this, you'll need to install the required Lua libraries:

1. **Install Lua Libraries**:
   ```bash
   sudo apt-get install liblua5.2-dev
   ```

   If you've already installed a Lua version other than 5.2, there might be conflicts. Ensure that version 5.2 is the one being detected or manually point CMake to the appropriate paths.

2. **Retry Building Cartographer**:
   Navigate back to your Cartographer workspace and try building again:
   ```bash
   cd /home/dan/cart_ws
   catkin_make_isolated --install
   ```

3. **Manually Indicate Lua Path** (if necessary):

   If you have Lua 5.2 installed, but CMake still cannot find it, you can manually point to its path using the `LUA_INCLUDE_DIR` and `LUA_LIBRARIES` variables:

   ```bash
   cd /home/dan/cart_ws
   catkin_make_isolated --install -DLUA_INCLUDE_DIR=/path/to/lua/include -DLUA_LIBRARIES=/path/to/lua/lib
   ```

   Replace `/path/to/lua/include` and `/path/to/lua/lib` with the correct paths where Lua headers and libraries are located.

Proceed with these steps based on your current setup, and then try building Cartographer again. If the issue persists or if you encounter other errors, please provide additional information.

## 4 
User
[ WARN] [1694528467.599063178]: MessageFilter [target=odom ]: Dropped 100.00% of messages so far. Please turn the [ros.gmapping.message_filter] rosconsole logger to DEBUG for more information.

The warning you're seeing indicates that the `gmapping` node's internal `MessageFilter` is dropping all the messages intended for the target frame `odom`. This is typically because the necessary transformations are not available in a timely manner.

Here are steps you can take to debug and resolve the issue:

1. **Enable DEBUG Logging**:
   - As suggested by the warning, turn on DEBUG logging for `ros.gmapping.message_filter`:
     ```bash
     rosconsole set /ros.gmapping.message_filter DEBUG
     ```
     This should give you more verbose output that will help in diagnosing the exact issue.

2. **Check TF Tree**:
   - Ensure that the transformation between `odom` and the laser's frame is being published.
     ```bash
     rosrun tf view_frames
     ```
     Examine the generated `frames.pdf` to ensure all expected transformations are present.

3. **Check Timestamps**:
   - Ensure that the timestamps on your messages (like `/scan` and `/odom`) are consistent and correct. Large discrepancies can cause TF to drop messages.

4. **Check TF Broadcast Rate**:
   - If transforms are being published too infrequently, it could lead to dropped messages. Ensure that any nodes responsible for broadcasting the relevant TFs are doing so at a suitable rate.

5. **Inspect the `odom` Topic**:
   - Confirm that the `/odom` topic is actively publishing data with the correct timestamps.
     ```bash
     rostopic echo /odom
     ```

6. **Launch Order**:
   - Ensure you launch nodes in the right order. Nodes responsible for broadcasting TFs (like `robot_state_publisher`) should typically be launched before nodes like `gmapping`.

7. **Buffer Size**:
   - The `MessageFilter` in ROS works with a buffer of transformations to allow for some delay in TF publishing. If your system has significant delays, you might need to adjust the buffer size. However, this would typically be done in the node's code or through a parameter, and it's more of an advanced adjustment. 

8. **Odometry Source**:
   - Confirm that your odometry source is functioning correctly. If you're simulating the robot, tools like Gazebo sometimes can introduce delays or issues with odometry.

9. **Network Issues**:
   - If you're working with multiple machines, ensure they are properly networked, the `ROS_MASTER_URI` is set correctly on all of them, and the system clocks are synchronized (using tools like `chrony` or `ntp`).

By systematically working through these suggestions, you should be able to identify the root cause and ensure that the necessary transformations are available for `gmapping` to operate correctly.

## 5
I know, the laserscan msg topic's frame is camera_init, but gmapping's fixed frame is map, so I think I need to alter /map into /camera_init, but how?

If the frame of your laserscan messages is `camera_init` but `gmapping` expects data in the `map` frame, you will need to set up appropriate transformations using `tf` (the ROS transform library). The goal is to ensure there's a coherent transformation chain between `camera_init` and `odom`, as `gmapping` also uses odometry data in its SLAM process.

However, if you simply want to change the fixed frame in `gmapping` from `map` to `camera_init`, you can do this by setting the appropriate parameter when launching the `gmapping` node. The parameter that determines the fixed frame in `gmapping` is `~map_frame`.

Here's how you can set it:

```bash
rosrun gmapping slam_gmapping scan:=/scan _map_frame:=camera_init
```

This will instruct `gmapping` to use `camera_init` as its fixed frame. 

That said, be cautious: simply changing this parameter without ensuring the correct transformations exist in your system could cause unexpected behavior. In a typical setup, the `map` frame is used as a fixed frame representing the map's global coordinate system, while the `odom` frame provides a local odometry-based reference for the robot. If you change the fixed frame to `camera_init`, make sure other components of your system (e.g., visualization tools, navigation components) are also aware of this change.

If you're still having trouble after making this change, ensure that:
1. A transformation exists from `camera_init` to your robot's base frame (e.g., `base_link`).
2. A transformation exists from your robot's base frame to the `odom` frame (usually provided by your robot's odometry system).

This ensures that `gmapping` can properly integrate both the laser scans and odometry information. If needed, static transforms can be set up using `static_transform_publisher` for any fixed relationships between frames.

## 6
   <node name="playbag" pkg="rosbag" type="play"
        args="--clock $(arg bag_filename)" /> If I pulish topic by bag file, show I add --clock args? The error I've met before can be boiled down to I did't use the bag file itself's timestamp

        Yes, when using ROS bag files for playback and simulations, especially when dealing with nodes that require accurate timing (like SLAM nodes), it's important to ensure the system uses the bag's timestamps rather than the current system time.

The `--clock` argument you've added in the rosbag play command makes the bag act as the "time source" for ROS. This essentially means that ROS will use the timestamps within the bag as the current time. This is achieved by publishing simulated clock messages on the `/clock` topic.

When you use the `--clock` option, it's also necessary to tell ROS to use this simulated clock. You do this by setting the `use_sim_time` parameter to `true`.

Before running your nodes (or in the launch file before playing back the bag file):

```xml
<param name="/use_sim_time" value="true"/>
```

This ensures that any nodes you run will use the simulated time provided by the bag. If you don't set this parameter, nodes will use the system clock, which will likely lead to the timestamp issues you experienced earlier.

To summarize:

1. Play the bag with the `--clock` option to publish simulated time on the `/clock` topic.
2. Set the `use_sim_time` parameter to `true` so ROS nodes will use the simulated time.

## 7
User [ WARN] [1694573592.625932696, 1694166602.118866841]: Failed to compute laser pose, aborting initialization ("base_link" passed to lookupTransform argument target_frame does not exist.

Following these steps should help in resolving issues related to timestamps when working with bag files.

## 8
(base) dan@dan-RedmiBook-Pro-14S:~$ rosrun tf view_frames 
Listening to /tf for 5.0 seconds
Done Listening
b'dot - graphviz version 2.43.0 (0)\n'
Traceback (most recent call last):
  File "/opt/ros/noetic/lib/tf/view_frames", line 119, in <module>
    generate(dot_graph)
  File "/opt/ros/noetic/lib/tf/view_frames", line 89, in generate
    m = r.search(vstr)
TypeError: cannot use a string pattern on a bytes-like object
[ WARN] [1694573592.625932696, 1694166602.118866841]: Failed to compute laser pose, aborting initialization ("base_link" passed to lookupTransform argument target_frame does not exist. 



The error you're seeing is related to the string encoding and how Python 3 (which ROS Noetic uses) handles strings and bytes. The output from `dot` is a bytes-like object, and Python 3 differentiates between strings (text) and bytes (binary data).

To resolve this, you have a few options:

1. **Quick Fix**:
   
   Edit the problematic script to handle the bytes output from `dot`.

   Open the script with:
   ```bash
   sudo nano /opt/ros/noetic/lib/tf/view_frames
   ```

   And find the line:
   ```python
   m = r.search(vstr)
   ```

   Change it to:
   ```python
   m = r.search(vstr.decode('utf-8'))
   ```

   This decodes the bytes-like object to a string using UTF-8 encoding.

   Save and exit (`Ctrl` + `O`, `Enter`, `Ctrl` + `X`).

2. **Long-term Fix**:

   The long-term solution would be to get an update from the maintainers of the `tf` package where this issue has been fixed. This can be done by checking if there's an update available for the ROS distribution you're using:
   
   ```bash
   sudo apt-get update
   sudo apt-get upgrade
   ```

3. **Alternative Visualization**:

   In case you don't want to edit the script directly, you can use `tf_monitor` to get information about the transforms:
   
   ```bash
   rosrun tf tf_monitor
   ```

   It doesn't provide a graphical view like `view_frames`, but it can give you information about which transforms are available, their rates, etc.

Once you've applied one of these solutions, you should be able to see the TF tree without any issues.


Following these steps should help in resolving issues related to timestamps when working with bag files.
