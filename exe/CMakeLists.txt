add_executable(runSimulator runSimulator.cpp)
target_link_libraries(runSimulator simulator exitRoom)

add_executable(offline_orb_slam offline_orb_slam.cc)
target_link_libraries(offline_orb_slam ${PROJECT_NAME})

add_executable(mapping mapping.cc)
target_link_libraries(mapping ${PROJECT_NAME})

#add_executable(simulator simulator.cc)
#target_link_libraries(simulator ${PROJECT_NAME})

add_executable(run_model run_model.cc)
target_link_libraries(run_model ${PROJECT_NAME})

add_executable(run_model_with_printed_orbs run_model_with_printed_orbs.cc)
target_link_libraries(run_model_with_printed_orbs ${PROJECT_NAME})

add_executable(run_model_with_icp run_model_with_icp.cc)
target_link_libraries(run_model_with_icp ${PROJECT_NAME})

add_executable(run_model_on_orb_slam run_model_on_orb_slam.cc)
target_link_libraries(run_model_on_orb_slam ${PROJECT_NAME})

add_executable(save_map save_map.cc)
target_link_libraries(save_map ${PROJECT_NAME})

add_executable(run_model_with_orb_slam_points run_model_with_orb_slam_points.cc)
target_link_libraries(run_model_with_orb_slam_points ${PROJECT_NAME})

add_executable(run_model_with_orbs_and_orb_slam_map run_model_with_orbs_and_orb_slam_map.cc)
target_link_libraries(run_model_with_orbs_and_orb_slam_map ${PROJECT_NAME})

add_executable(test_exitRoomAlgo test_exitRoomAlgo.cpp)
target_link_libraries(test_exitRoomAlgo exitRoom)
