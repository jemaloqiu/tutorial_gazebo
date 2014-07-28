cmake_minimum_required(VERSION 2.8.3)
project(wifibot_controller)




find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs)

find_package(catkin REQUIRED COMPONENTS 
  gazebo_plugins
  gazebo_ros
)

include_directories(/usr/local/include/gazebo-1.9)
include_directories(/usr/include/sdformat-1.4)

find_package(Boost REQUIRED COMPONENTS thread)


# Find gazebo
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

include_directories(include 
  ${catkin_INCLUDE_DIRS} 
  ${GAZEBO_INCLUDE_DIRS}
)

catkin_package(
    DEPENDS 
      eigen
      Boost
    CATKIN_DEPENDS
      gazebo_plugins 
      gazebo_ros
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME} 
) 

add_library(WifibotController src/wifibotController.cpp)
# target_link_libraries(WifibotController thread)



