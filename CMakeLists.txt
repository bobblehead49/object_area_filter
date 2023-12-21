cmake_minimum_required(VERSION 3.0.2)
project(object_area_filter)

find_package(catkin REQUIRED 
  roscpp
  tf2_ros
  tf2_geometry_msgs
  object_msgs
  point_in_polygon_lib
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(object_area_filter src/object_area_filter.cpp)
target_link_libraries(object_area_filter ${catkin_LIBRARIES})
target_compile_options(object_area_filter PUBLIC -O3 -Wall)

add_executable(object_area_filter_node src/object_area_filter_node.cpp)
target_link_libraries(object_area_filter_node ${catkin_LIBRARIES} object_area_filter)
target_compile_options(object_area_filter_node PUBLIC -O3 -Wall)

install(TARGETS object_area_filter
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS object_area_filter_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
