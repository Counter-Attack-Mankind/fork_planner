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
  sandbox_msgs
)
find_package(casadi CONFIG REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES forklift_nlp_recovery_geometry forklift_nlp_recovery_input forklift_nlp_recovery_initial_guess forklift_nlp_recovery_solver forklift_nlp_recovery_replay
  CATKIN_DEPENDS roscpp roslib geometry_msgs std_msgs std_srvs visualization_msgs forklift_map sandbox_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(forklift_nlp_recovery_geometry src/corridor_builder.cpp)
add_dependencies(forklift_nlp_recovery_geometry ${catkin_EXPORTED_TARGETS})
target_link_libraries(forklift_nlp_recovery_geometry ${catkin_LIBRARIES})

add_library(forklift_nlp_recovery_input src/optimization_input.cpp)
add_dependencies(forklift_nlp_recovery_input ${catkin_EXPORTED_TARGETS})
target_link_libraries(forklift_nlp_recovery_input
  forklift_nlp_recovery_geometry
  ${catkin_LIBRARIES}
)

add_library(forklift_nlp_recovery_initial_guess src/initial_guess.cpp)
add_dependencies(forklift_nlp_recovery_initial_guess ${catkin_EXPORTED_TARGETS})
target_link_libraries(forklift_nlp_recovery_initial_guess
  forklift_nlp_recovery_input
  ${catkin_LIBRARIES}
)

add_library(forklift_nlp_recovery_solver src/nlp_solver.cpp)
add_dependencies(forklift_nlp_recovery_solver ${catkin_EXPORTED_TARGETS})
target_link_libraries(forklift_nlp_recovery_solver
  forklift_nlp_recovery_initial_guess
  casadi
  ${catkin_LIBRARIES}
)

add_library(forklift_nlp_recovery_replay src/replay_data.cpp)
add_dependencies(forklift_nlp_recovery_replay ${catkin_EXPORTED_TARGETS})
target_link_libraries(forklift_nlp_recovery_replay yaml-cpp ${catkin_LIBRARIES})

add_executable(nlp_trajectory_replay_node src/nlp_trajectory_replay_node.cpp)
add_dependencies(nlp_trajectory_replay_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(nlp_trajectory_replay_node
  forklift_nlp_recovery_replay
  forklift_map_lib
  ${catkin_LIBRARIES}
)

add_executable(recovery_visualizer_node src/recovery_visualizer_node.cpp)
add_dependencies(recovery_visualizer_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(recovery_visualizer_node
  forklift_nlp_recovery_solver
  forklift_nlp_recovery_initial_guess
  forklift_nlp_recovery_input
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
                forklift_nlp_recovery_solver
                forklift_nlp_recovery_replay
                recovery_visualizer_node
                nlp_trajectory_replay_node
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
