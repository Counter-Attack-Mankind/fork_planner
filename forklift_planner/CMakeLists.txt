cmake_minimum_required(VERSION 3.5)
project(forklift_planner)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-O2)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  visualization_msgs
  geometry_msgs
  std_msgs
  forklift_map
  sandbox_msgs   # 实车桥:AprilObject(位姿入) / Trajectory(轨迹出)
)

catkin_package(
  CATKIN_DEPENDS roscpp roslib visualization_msgs geometry_msgs std_msgs forklift_map sandbox_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(PATH_GENERATOR_SOURCES
  src/path_generator.cpp
  src/path_generator_internal.cpp
  src/path_generator_route.cpp
  src/path_generator_routes/path_generator_route_dispatch.cpp
  src/path_generator_routes/a1_to_b/path_generator_route.cpp
  src/path_generator_routes/b_to_a1/path_generator_route.cpp
  src/path_generator_routes/a2_to_b/path_generator_route.cpp
  src/path_generator_routes/b_to_a2/path_generator_route.cpp
)

add_executable(planner_node
  ${PATH_GENERATOR_SOURCES}
  src/planner_node.cpp
)
add_dependencies(planner_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(planner_node
  ${catkin_LIBRARIES}
)

add_executable(two_vehicle_sim_node
  ${PATH_GENERATOR_SOURCES}
  src/two_vehicle_sim_node.cpp
)
add_dependencies(two_vehicle_sim_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(two_vehicle_sim_node
  ${catkin_LIBRARIES}
)

add_executable(multi_vehicle_patrol_node
  ${PATH_GENERATOR_SOURCES}
  src/multi_vehicle/footprint.cpp
  src/multi_vehicle/bridge_ttc_correction.cpp
  src/multi_vehicle/dynamic_speed_coordination.cpp
  src/multi_vehicle/marker_publisher.cpp
  src/multi_vehicle/multi_vehicle_config.cpp
  src/multi_vehicle/path_track.cpp
  src/multi_vehicle/a1/a1_coordinator.cpp
  src/multi_vehicle/deadlock/deadlock_manager.cpp
  src/multi_vehicle/rule_engine.cpp
  src/multi_vehicle/spatiotemporal_interaction.cpp
  src/multi_vehicle/task_allocator.cpp
  src/multi_vehicle/traffic_resource_map.cpp
  src/multi_vehicle_patrol_node.cpp
)
add_dependencies(multi_vehicle_patrol_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(multi_vehicle_patrol_node
  ${catkin_LIBRARIES}
)

catkin_install_python(PROGRAMS
  scripts/realbridge_controller_launcher.py
  scripts/realbridge_keyboard_control.py
  scripts/rolling_rviz_bag_recorder.py
  scripts/replay_rviz_snapshot.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 注:实车桥已并入 multi_vehicle_patrol_node(real_mode 开关,一份代码两个模式),
# 独立的 multi_vehicle_realbridge_node 已退役(避免重复实现走样)。

# [DIAG] standalone path-generator probe (no ROS runtime)
add_executable(path_debug
  ${PATH_GENERATOR_SOURCES}
  src/multi_vehicle/footprint.cpp
  src/path_debug.cpp
)
add_dependencies(path_debug ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_debug ${catkin_LIBRARIES})

# Read-only B <-> A1 final-path curvature audit. It reuses the production
# route generators and writes diagnostics only; it is not part of runtime
# planning or control.
add_executable(path_curvature_audit
  ${PATH_GENERATOR_SOURCES}
  src/multi_vehicle/footprint.cpp
  src/multi_vehicle/multi_vehicle_config.cpp
  src/path_generator_routes/path_curvature_audit.cpp
)
add_dependencies(path_curvature_audit ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_curvature_audit ${catkin_LIBRARIES})

# [DIAG] EXP-002: offline ConflictZone geometry probe.  This executable does
# not participate in the patrol node or change coordination behavior.
add_executable(conflict_zone_geometry_diag
  ${PATH_GENERATOR_SOURCES}
  src/multi_vehicle/footprint.cpp
  src/multi_vehicle/path_track.cpp
  tools/conflict_zone_geometry_diag.cpp
)
add_dependencies(conflict_zone_geometry_diag ${catkin_EXPORTED_TARGETS})
target_link_libraries(conflict_zone_geometry_diag
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  add_executable(real_state_estimation_test
    test/real_state_estimation_test.cpp
    src/multi_vehicle/path_track.cpp
  )
  add_dependencies(real_state_estimation_test ${catkin_EXPORTED_TARGETS})
  target_link_libraries(real_state_estimation_test ${catkin_LIBRARIES})
  add_test(NAME real_state_estimation_test
           COMMAND ${CATKIN_DEVEL_PREFIX}/env.sh
                   $<TARGET_FILE:real_state_estimation_test>)



  add_executable(bridge_ttc_correction_test
    test/bridge_ttc_correction_test.cpp
    src/multi_vehicle/bridge_ttc_correction.cpp
    src/multi_vehicle/spatiotemporal_interaction.cpp
    src/multi_vehicle/footprint.cpp
    src/multi_vehicle/path_track.cpp
  )
  add_dependencies(bridge_ttc_correction_test ${catkin_EXPORTED_TARGETS})
  target_link_libraries(bridge_ttc_correction_test ${catkin_LIBRARIES})
  add_test(NAME bridge_ttc_correction_test
           COMMAND ${CATKIN_DEVEL_PREFIX}/env.sh
                   $<TARGET_FILE:bridge_ttc_correction_test>)

  add_executable(conflict_zone_closure_test
    test/conflict_zone_closure_test.cpp
  )
  add_test(NAME conflict_zone_closure_test COMMAND conflict_zone_closure_test)

  add_executable(future_a1_policy_test
    test/future_a1_policy_test.cpp
  )
  add_test(NAME future_a1_policy_test COMMAND future_a1_policy_test)

  add_executable(spatiotemporal_interaction_test
    test/spatiotemporal_interaction_test.cpp
    src/multi_vehicle/bridge_ttc_correction.cpp
    src/multi_vehicle/dynamic_speed_coordination.cpp
    src/multi_vehicle/footprint.cpp
    src/multi_vehicle/path_track.cpp
    src/multi_vehicle/a1/a1_coordinator.cpp
    src/multi_vehicle/deadlock/deadlock_manager.cpp
    src/multi_vehicle/rule_engine.cpp
    src/multi_vehicle/spatiotemporal_interaction.cpp
    src/multi_vehicle/traffic_resource_map.cpp
  )
  add_dependencies(spatiotemporal_interaction_test ${catkin_EXPORTED_TARGETS})
  target_link_libraries(spatiotemporal_interaction_test ${catkin_LIBRARIES})
  add_test(NAME spatiotemporal_interaction_test
           COMMAND ${CATKIN_DEVEL_PREFIX}/env.sh
                   $<TARGET_FILE:spatiotemporal_interaction_test>)

  add_executable(dynamic_speed_coordination_test
    test/dynamic_speed_coordination_test.cpp
    src/multi_vehicle/bridge_ttc_correction.cpp
    src/multi_vehicle/dynamic_speed_coordination.cpp
    src/multi_vehicle/footprint.cpp
    src/multi_vehicle/path_track.cpp
    src/multi_vehicle/spatiotemporal_interaction.cpp
  )
  add_dependencies(dynamic_speed_coordination_test ${catkin_EXPORTED_TARGETS})
  target_link_libraries(dynamic_speed_coordination_test ${catkin_LIBRARIES})
  add_test(NAME dynamic_speed_coordination_test
           COMMAND ${CATKIN_DEVEL_PREFIX}/env.sh
                   $<TARGET_FILE:dynamic_speed_coordination_test>)


  add_executable(prediction_execution_consistency_test
    test/prediction_execution_consistency_test.cpp
    src/multi_vehicle/footprint.cpp
    src/multi_vehicle/path_track.cpp
    src/multi_vehicle/spatiotemporal_interaction.cpp
  )
  add_dependencies(prediction_execution_consistency_test
                   ${catkin_EXPORTED_TARGETS})
  target_link_libraries(prediction_execution_consistency_test
                        ${catkin_LIBRARIES})
  add_test(NAME prediction_execution_consistency_test
           COMMAND ${CATKIN_DEVEL_PREFIX}/env.sh
                   $<TARGET_FILE:prediction_execution_consistency_test>)

  add_executable(rolling_decision_timing_test
    test/rolling_decision_timing_test.cpp
    src/multi_vehicle/bridge_ttc_correction.cpp
    src/multi_vehicle/dynamic_speed_coordination.cpp
    src/multi_vehicle/footprint.cpp
    src/multi_vehicle/path_track.cpp
    src/multi_vehicle/a1/a1_coordinator.cpp
    src/multi_vehicle/deadlock/deadlock_manager.cpp
    src/multi_vehicle/rule_engine.cpp
    src/multi_vehicle/spatiotemporal_interaction.cpp
    src/multi_vehicle/traffic_resource_map.cpp
  )
  add_dependencies(rolling_decision_timing_test ${catkin_EXPORTED_TARGETS})
  target_link_libraries(rolling_decision_timing_test ${catkin_LIBRARIES})
  add_test(NAME rolling_decision_timing_test
           COMMAND ${CATKIN_DEVEL_PREFIX}/env.sh
                   $<TARGET_FILE:rolling_decision_timing_test>)

endif()

install(TARGETS planner_node two_vehicle_sim_node multi_vehicle_patrol_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
