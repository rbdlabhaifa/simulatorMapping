add_executable(runSimulator runSimulator.cpp)
target_link_libraries(runSimulator simulator exitRoom)

add_executable(runPolySimulator runPolySimulator.cpp)
target_link_libraries(runPolySimulator polySimulator simulatorManager ${PROJECT_NAME})

add_executable(offline_orb_slam offline_orb_slam.cc)
target_link_libraries(offline_orb_slam ${PROJECT_NAME})

add_executable(mapping mapping.cc)
target_link_libraries(mapping ${PROJECT_NAME})

add_executable(save_map save_map.cc)
target_link_libraries(save_map ${PROJECT_NAME})

add_executable(save_first_frame save_first_frame.cc)
target_link_libraries(save_first_frame ${PROJECT_NAME})

add_executable(save_keypoints_mapping save_keypoints_mapping.cc)
target_link_libraries(save_keypoints_mapping ${PROJECT_NAME})

add_executable(remove_map_points remove_map_points.cc)
target_link_libraries(remove_map_points ${PROJECT_NAME})

add_executable(navigate_to_point navigate_to_point.cc)
target_link_libraries(navigate_to_point ${PROJECT_NAME})

add_executable(demo demo.cc)
target_link_libraries(demo ${PROJECT_NAME})

add_executable(build_new_map_demo build_new_map_demo.cc)
target_link_libraries(build_new_map_demo ${PROJECT_NAME})

add_executable(test_exitRoomAlgo test_exitRoomAlgo.cpp)
target_link_libraries(test_exitRoomAlgo exitRoom)
