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
