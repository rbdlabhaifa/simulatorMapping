add_executable(runSimulator runSimulator.cpp)
target_link_libraries(runSimulator simulator exitRoom Auxiliary)

add_executable(offline_orb_slam offline_orb_slam.cc)
target_link_libraries(offline_orb_slam ORB_SLAM2 Auxiliary)

add_executable(mapping mapping.cc)
target_link_libraries(mapping ORB_SLAM2 Auxiliary)

add_executable(navigate_to_point navigate_to_point.cc)
target_link_libraries(navigate_to_point ORB_SLAM2 Auxiliary simulator)

add_executable(getExitPoints getExitPoints.cpp)
target_link_libraries(getExitPoints exitRoom)
