<<<<<<< HEAD
<<<<<<< HEAD
# Install script for directory: /home/dan/train/train/MA_training/summuer_training_1
=======
# Install script for directory: /home/dan/train/train/MA_training/MA_training/summuer_training_1
>>>>>>> b4fbe88 (Add deep learning demo)
=======
# Install script for directory: /home/dan/train/train/MA_training/MA_training/summuer_training_1
>>>>>>> 21002ca (Add some openvino demo)

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
<<<<<<< HEAD
<<<<<<< HEAD
  include("/home/dan/train/train/MA_training/summuer_training_1/build/modules/cmake_install.cmake")
=======
  include("/home/dan/train/train/MA_training/MA_training/summuer_training_1/build/modules/cmake_install.cmake")
>>>>>>> b4fbe88 (Add deep learning demo)
=======
  include("/home/dan/train/train/MA_training/MA_training/summuer_training_1/build/modules/cmake_install.cmake")
>>>>>>> 21002ca (Add some openvino demo)
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
<<<<<<< HEAD
<<<<<<< HEAD
file(WRITE "/home/dan/train/train/MA_training/summuer_training_1/build/${CMAKE_INSTALL_MANIFEST}"
=======
file(WRITE "/home/dan/train/train/MA_training/MA_training/summuer_training_1/build/${CMAKE_INSTALL_MANIFEST}"
>>>>>>> b4fbe88 (Add deep learning demo)
=======
file(WRITE "/home/dan/train/train/MA_training/MA_training/summuer_training_1/build/${CMAKE_INSTALL_MANIFEST}"
>>>>>>> 21002ca (Add some openvino demo)
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
