cmake_minimum_required(VERSION 3.5)
project(forklift_nlp_recovery)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-O2 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  visualization_msgs
  forklift_map
  sandbox_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES forklift_nlp_recovery_geometry
  CATKIN_DEPENDS roscpp geometry_msgs std_msgs visualization_msgs forklift_map sandbox_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(forklift_nlp_recovery_geometry src/corridor_builder.cpp)
add_dependencies(forklift_nlp_recovery_geometry ${catkin_EXPORTED_TARGETS})
target_link_libraries(forklift_nlp_recovery_geometry ${catkin_LIBRARIES})

add_executable(recovery_visualizer_node src/recovery_visualizer_node.cpp)
add_dependencies(recovery_visualizer_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(recovery_visualizer_node
  forklift_nlp_recovery_geometry
  forklift_map_lib
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(corridor_builder_test test/corridor_builder_test.cpp)
  if(TARGET corridor_builder_test)
    add_dependencies(corridor_builder_test ${catkin_EXPORTED_TARGETS})
    target_link_libraries(corridor_builder_test
      forklift_nlp_recovery_geometry
      forklift_map_lib
      ${catkin_LIBRARIES}
    )
  endif()
endif()

install(TARGETS forklift_nlp_recovery_geometry recovery_visualizer_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY config launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
