add_executable(runSimulator runSimulator.cpp)
target_link_libraries(runSimulator simulator exitRoom Auxiliary)

add_executable(offline_orb_slam offline_orb_slam.cc)
target_link_libraries(offline_orb_slam ORB_SLAM2 Auxiliary)

add_executable(mapping mapping.cc)
target_link_libraries(mapping ORB_SLAM2 Auxiliary)

add_executable(save_map save_map.cc)
target_link_libraries(save_map ORB_SLAM2 Auxiliary)

add_executable(save_first_frame save_first_frame.cc)
target_link_libraries(save_first_frame ORB_SLAM2 Auxiliary MapControl)

add_executable(save_keypoints_mapping save_keypoints_mapping.cc)
target_link_libraries(save_keypoints_mapping ORB_SLAM2 Auxiliary)

add_executable(remove_map_points remove_map_points.cc)
target_link_libraries(remove_map_points ORB_SLAM2 Auxiliary)

add_executable(navigate_to_point navigate_to_point.cc)
target_link_libraries(navigate_to_point ORB_SLAM2 Auxiliary)

add_executable(demo demo.cc)
target_link_libraries(demo ORB_SLAM2 MapControl)

add_executable(testExitRoom testExitRoom.cpp)
target_link_libraries(testExitRoom exitRoom)
