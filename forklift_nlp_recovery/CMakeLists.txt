cmake_minimum_required(VERSION 3.5)
project(forklift_nlp_recovery)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-O2 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  geometry_msgs
  std_msgs
  std_srvs
  visualization_msgs
  forklift_map
  forklift_planner
  sandbox_msgs
)
find_package(casadi CONFIG REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES forklift_nlp_recovery_geometry forklift_nlp_recovery_input forklift_nlp_recovery_initial_guess forklift_nlp_recovery_validator forklift_nlp_recovery_constraint_diagnostics forklift_nlp_recovery_solver forklift_nlp_recovery_replay
  CATKIN_DEPENDS roscpp roslib geometry_msgs std_msgs std_srvs visualization_msgs forklift_map forklift_planner sandbox_msgs
)

include_directories(include ../forklift_planner/include ${catkin_INCLUDE_DIRS})

add_library(forklift_nlp_recovery_geometry
  src/optimization/corridor_builder.cpp
  src/geometry/coordinate_mapper.cpp
)
add_dependencies(forklift_nlp_recovery_geometry ${catkin_EXPORTED_TARGETS})
target_link_libraries(forklift_nlp_recovery_geometry ${catkin_LIBRARIES})

add_library(forklift_nlp_recovery_input src/optimization/optimization_input.cpp)
add_dependencies(forklift_nlp_recovery_input ${catkin_EXPORTED_TARGETS})
target_link_libraries(forklift_nlp_recovery_input
  forklift_nlp_recovery_geometry
  ${catkin_LIBRARIES}
)

add_library(forklift_nlp_recovery_initial_guess
  src/optimization/initial_guess.cpp
)
add_dependencies(forklift_nlp_recovery_initial_guess ${catkin_EXPORTED_TARGETS})
target_link_libraries(forklift_nlp_recovery_initial_guess
  forklift_nlp_recovery_input
  ${catkin_LIBRARIES}
)

add_library(forklift_nlp_recovery_validator src/optimization/nlp_validator.cpp)
add_dependencies(forklift_nlp_recovery_validator ${catkin_EXPORTED_TARGETS})
target_link_libraries(forklift_nlp_recovery_validator
  forklift_nlp_recovery_initial_guess
  ${catkin_LIBRARIES}
)

add_library(forklift_nlp_recovery_constraint_diagnostics
  src/optimization/nlp_constraint_diagnostics.cpp
)
add_dependencies(forklift_nlp_recovery_constraint_diagnostics ${catkin_EXPORTED_TARGETS})
target_link_libraries(forklift_nlp_recovery_constraint_diagnostics ${catkin_LIBRARIES})

add_library(forklift_nlp_recovery_solver src/optimization/nlp_solver.cpp)
add_dependencies(forklift_nlp_recovery_solver ${catkin_EXPORTED_TARGETS})
target_link_libraries(forklift_nlp_recovery_solver
  forklift_nlp_recovery_initial_guess
  forklift_nlp_recovery_validator
  forklift_nlp_recovery_constraint_diagnostics
  casadi
  ${catkin_LIBRARIES}
)

add_library(forklift_nlp_recovery_replay src/visualization/replay_data.cpp)
add_dependencies(forklift_nlp_recovery_replay ${catkin_EXPORTED_TARGETS})
target_link_libraries(forklift_nlp_recovery_replay yaml-cpp ${catkin_LIBRARIES})

add_executable(nlp_trajectory_replay_node src/visualization/nlp_trajectory_replay_node.cpp)
add_dependencies(nlp_trajectory_replay_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(nlp_trajectory_replay_node
  forklift_nlp_recovery_replay
  forklift_map_lib
  ${catkin_LIBRARIES}
)

add_executable(recovery_visualizer_node
  src/visualization/recovery_visualizer_node.cpp
  src/visualization/recovery_stage_visualization.cpp
)
add_dependencies(recovery_visualizer_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(recovery_visualizer_node
  forklift_nlp_recovery_solver
  forklift_nlp_recovery_initial_guess
  forklift_nlp_recovery_input
  forklift_nlp_recovery_geometry
  forklift_map_lib
  ${catkin_LIBRARIES}
)

set(COORDINATE_MAPPING_PATH_GENERATOR_SOURCES
  ../forklift_planner/src/path_generator.cpp
  ../forklift_planner/src/path_generator_internal.cpp
  ../forklift_planner/src/path_generator_route.cpp
  ../forklift_planner/src/path_generator_routes/path_generator_route_dispatch.cpp
  ../forklift_planner/src/path_generator_routes/a1_to_b/path_generator_route.cpp
  ../forklift_planner/src/path_generator_routes/b_to_a1/path_generator_route.cpp
  ../forklift_planner/src/path_generator_routes/a2_to_b/path_generator_route.cpp
  ../forklift_planner/src/path_generator_routes/b_to_a2/path_generator_route.cpp
)
add_executable(coordinate_mapping_visualizer_node
  src/visualization/coordinate_mapping_visualizer_node.cpp
  ${COORDINATE_MAPPING_PATH_GENERATOR_SOURCES}
)
add_dependencies(coordinate_mapping_visualizer_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(coordinate_mapping_visualizer_node
  forklift_nlp_recovery_geometry
  forklift_map_lib
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(coordinate_mapper_test test/coordinate_mapper_test.cpp)
  if(TARGET coordinate_mapper_test)
    add_dependencies(coordinate_mapper_test ${catkin_EXPORTED_TARGETS})
    target_link_libraries(coordinate_mapper_test
      forklift_nlp_recovery_geometry
      ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(corridor_builder_test test/corridor_builder_test.cpp)
  if(TARGET corridor_builder_test)
    add_dependencies(corridor_builder_test ${catkin_EXPORTED_TARGETS})
    target_link_libraries(corridor_builder_test
      forklift_nlp_recovery_geometry
      forklift_map_lib
      ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(optimization_input_test test/optimization_input_test.cpp)
  if(TARGET optimization_input_test)
    add_dependencies(optimization_input_test ${catkin_EXPORTED_TARGETS})
    target_link_libraries(optimization_input_test
      forklift_nlp_recovery_input
      forklift_nlp_recovery_geometry
      forklift_map_lib
      ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(initial_guess_test test/initial_guess_test.cpp)
  if(TARGET initial_guess_test)
    add_dependencies(initial_guess_test ${catkin_EXPORTED_TARGETS})
    target_link_libraries(initial_guess_test
      forklift_nlp_recovery_initial_guess
      forklift_nlp_recovery_input
      forklift_nlp_recovery_geometry
      ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(nlp_solver_test test/nlp_solver_test.cpp)
  if(TARGET nlp_solver_test)
    add_dependencies(nlp_solver_test ${catkin_EXPORTED_TARGETS})
    target_link_libraries(nlp_solver_test
      forklift_nlp_recovery_solver
      forklift_nlp_recovery_initial_guess
      forklift_nlp_recovery_input
      forklift_nlp_recovery_geometry
      casadi
      ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(replay_data_test test/replay_data_test.cpp)
  if(TARGET replay_data_test)
    add_dependencies(replay_data_test ${catkin_EXPORTED_TARGETS})
    target_link_libraries(replay_data_test forklift_nlp_recovery_replay yaml-cpp ${catkin_LIBRARIES})
  endif()
endif()

install(TARGETS forklift_nlp_recovery_geometry forklift_nlp_recovery_input
                forklift_nlp_recovery_initial_guess
                forklift_nlp_recovery_validator
                forklift_nlp_recovery_constraint_diagnostics
                forklift_nlp_recovery_solver
                forklift_nlp_recovery_replay
                recovery_visualizer_node
                nlp_trajectory_replay_node
                coordinate_mapping_visualizer_node
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
install(DIRECTORY results/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/results
  PATTERN "case_*" EXCLUDE
  PATTERN ".git*" EXCLUDE
)
