cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME droneObstacleAvoiderROSModule)
project(${PROJECT_NAME})

### Use version 2011 of C++ (c++11). By default ROS uses c++98
add_definitions(-std=c++11)

set(DRONE_OBSTACLE_AVOIDER_ROS_SOURCE_DIR
	src/source) 
	
set(DRONE_OBSTACLE_AVOIDER_ROS_INCLUDE_DIR
	src/include)

set(DRONE_OBSTACLE_AVOIDER_ROS_SOURCE_FILES
        ${DRONE_OBSTACLE_AVOIDER_ROS_SOURCE_DIR}/droneObstacleAvoiderROSModule.cpp

)
 
set(DRONE_OBSTACLE_AVOIDER_ROS_HEADER_FILES
        ${DRONE_OBSTACLE_AVOIDER_ROS_INCLUDE_DIR}/droneObstacleAvoiderROSModule.h
)

find_package(
		catkin REQUIRED
        COMPONENTS
        roscpp
        std_msgs
        geometry_msgs
        droneModuleROS
        droneObstacleAvoider
        droneMidLevelAutopilot
        droneMsgsROS
)

catkin_package(
        CATKIN_DEPENDS
        roscpp
        std_msgs
        geometry_msgs
        droneModuleROS
        droneObstacleAvoider
        droneMidLevelAutopilot
        droneMsgsROS
)

include_directories(${DRONE_OBSTACLE_AVOIDER_ROS_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})

add_library(droneObstacleAvoiderROSModule ${DRONE_OBSTACLE_AVOIDER_ROS_SOURCE_FILES} ${DRONE_OBSTACLE_AVOIDER_ROS_HEADER_FILES})
add_dependencies(droneObstacleAvoiderROSModule ${catkin_EXPORTED_TARGETS})
target_link_libraries(droneObstacleAvoiderROSModule ${catkin_LIBRARIES})

add_executable(droneObstacleAvoiderROSModuleNode ${DRONE_OBSTACLE_AVOIDER_ROS_SOURCE_DIR}/droneObstacleAvoiderROSModuleNode.cpp)
add_dependencies(droneObstacleAvoiderROSModuleNode ${catkin_EXPORTED_TARGETS})
target_link_libraries(droneObstacleAvoiderROSModuleNode droneObstacleAvoiderROSModule)
target_link_libraries(droneObstacleAvoiderROSModuleNode ${catkin_LIBRARIES})

