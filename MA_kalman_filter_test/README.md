# MA

## If you met a bug named you are lack of -fPIC libfmt.a

Alter fmt source code CMakeLists.txt, add a compile command 
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
