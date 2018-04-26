cmake_minimum_required(VERSION 2.8.3)
project(perception)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    cv_bridge
    image_transport
    darknet_ros_msgs
    perception_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
 CATKIN_DEPENDS
   roscpp
   cv_bridge
   image_transport
   darknet_ros_msgs
   perception_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_read_image_node src/read_image_node.cpp)
add_executable(${PROJECT_NAME}_tracker_node src/tracker_node.cpp src/WorldPositioner.cpp)

add_dependencies(${PROJECT_NAME}_tracker_node perception_msgs_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_tracker_node darknet_ros_msgs_generate_messages_cpp)
MESSAGE("catkin_EXPORTED_TARGETS " ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_read_image_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_tracker_node ${catkin_LIBRARIES})
