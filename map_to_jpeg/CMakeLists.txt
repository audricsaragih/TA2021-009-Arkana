#cmake_minimum_required(VERSION 2.8.3)
cmake_minimum_required(VERSION 3.0.2)
project(map_to_jpeg)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS sensor_msgs nav_msgs geometry_msgs tf image_transport cv_bridge)

catkin_package(
   INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp
   DEPENDS eigen OpenCV
)

#include_directories(${EIGEN3_INCLUDE_DIRS} include ${catkin_INCLUDE_DIRS})
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(map_to_image_node src/map_to_image_node.cpp)
#target_link_libraries(map_to_image_node ${catkin_LIBRARIES})
#target_link_libraries(map_to_image_node ${OpenCV_LIBRARIES})
target_link_libraries(map_to_image_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(map_to_image_node_edit src/map_to_image_node_edit.cpp)
#target_link_libraries(map_to_image_node ${catkin_LIBRARIES})
#target_link_libraries(map_to_image_node ${OpenCV_LIBRARIES})
target_link_libraries(map_to_image_node_edit
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(map_to_image_node_edit_tile src/map_to_image_node_edit_tile.cpp)
#target_link_libraries(map_to_image_node ${catkin_LIBRARIES})
#target_link_libraries(map_to_image_node ${OpenCV_LIBRARIES})
target_link_libraries(map_to_image_node_edit_tile
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(image_to_map_node src/image_to_map_node.cpp)
target_link_libraries(image_to_map_node ${catkin_LIBRARIES})

#install(TARGETS map_to_image_node
  #ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  #LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  #RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
