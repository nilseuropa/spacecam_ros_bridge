cmake_minimum_required(VERSION 2.8.3)
project(spacecam_bridge)

add_compile_options(-std=c++11)

## enable profiling
#add_compile_options(-pg)
#set(catkin_LIBRARIES ${catkin_LIBRARIES} -pg)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  visualization_msgs
  image_transport
  cv_bridge
  tf
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES spacecam
   CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(trackpoint_publisher src/trackpoint_publisher_node.cpp)
target_link_libraries(trackpoint_publisher
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(opencv_posit src/opencv_posit_node.cpp)
target_link_libraries(opencv_posit
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(love_at_first_sight src/love_at_first_sight.cpp)
target_link_libraries(love_at_first_sight
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_library(POS
    include/pos.cpp
)

add_executable(cpp_posit src/cpp_posit_node.cpp)
target_link_libraries(cpp_posit POS
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES} # debug screen only
)

add_executable(cpp_smart_posit src/cpp_smart_posit_node.cpp)
target_link_libraries(cpp_smart_posit POS
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES} # debug screen only
)
