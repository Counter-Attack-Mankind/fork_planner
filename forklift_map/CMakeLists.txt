cmake_minimum_required(VERSION 3.5)
project(forklift_map)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-O2)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  geometry_msgs
  std_msgs
  tf2
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES forklift_map_lib
  CATKIN_DEPENDS roscpp visualization_msgs geometry_msgs std_msgs tf2
)

include_directories(
  include
  include/forklift_map            # for "common/math/vec2d.h" style includes
  include/forklift_map/common/math  # for relative "vec2d.h" inside copied headers
  ../forklift_planner/include
  ${catkin_INCLUDE_DIRS}
)

# Pure-logic map library (no ROS I/O)
add_library(forklift_map_lib
  src/forklift_map.cpp
  src/clothoid.cpp
)
add_dependencies(forklift_map_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(forklift_map_lib ${catkin_LIBRARIES})

# RViz visualization node
add_executable(map_node src/map_node.cpp)
add_dependencies(map_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(map_node
  forklift_map_lib
  ${catkin_LIBRARIES}
)

add_executable(path_catalog_debug_node
  ../forklift_planner/src/path_generator.cpp
  ../forklift_planner/src/path_generator_internal.cpp
  ../forklift_planner/src/path_generator_route.cpp
  ../forklift_planner/src/path_generator_routes/path_generator_route_dispatch.cpp
  ../forklift_planner/src/path_generator_routes/a1_to_b/path_generator_route.cpp
  ../forklift_planner/src/path_generator_routes/b_to_a1/path_generator_route.cpp
  ../forklift_planner/src/path_generator_routes/a2_to_b/path_generator_route.cpp
  ../forklift_planner/src/path_generator_routes/b_to_a2/path_generator_route.cpp
  ../forklift_planner/src/multi_vehicle/footprint.cpp
  ../forklift_planner/src/multi_vehicle/multi_vehicle_config.cpp
  src/path_debug_visualizer.cpp
  src/path_catalog_debug_node.cpp
)
add_dependencies(path_catalog_debug_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_catalog_debug_node
  forklift_map_lib
  ${catkin_LIBRARIES}
)

install(TARGETS map_node path_catalog_debug_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
