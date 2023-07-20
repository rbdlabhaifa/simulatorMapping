add_executable(runSimulator runSimulator.cpp)
target_link_libraries(runSimulator simulator exitRoom)

add_executable(offline_orb_slam offline_orb_slam.cc)
target_link_libraries(offline_orb_slam ${PROJECT_NAME})

add_executable(get_point_data get_point_data.cc)
target_link_libraries(get_point_data ${PROJECT_NAME})

add_executable(unused_get_points_from_camera unused/get_points_from_camera.cc)
target_link_libraries(unused_get_points_from_camera ${PROJECT_NAME})

add_executable(unused_test_is_point_visible unused/test_is_point_visible.cc)
target_link_libraries(unused_test_is_point_visible ${PROJECT_NAME})

add_executable(unused_get_points_from_camera_second unused/get_points_from_camera_second.cc)
target_link_libraries(unused_get_points_from_camera_second ${PROJECT_NAME})

add_executable(unused_points_seen_by_camera_by_frame unused/points_seen_by_camera_by_frame.cc)
target_link_libraries(unused_points_seen_by_camera_by_frame ${PROJECT_NAME})

add_executable(unused_points_seen_by_camera_by_frame_second unused/points_seen_by_camera_by_frame_second.cc)
target_link_libraries(unused_points_seen_by_camera_by_frame_second ${PROJECT_NAME})

#add_executable(get_points_from_frame_orb_slam get_points_from_frame_orb_slam.cc)
#target_link_libraries(get_points_from_frame_orb_slam ${PROJECT_NAME})

add_executable(unused_get_points_from_camera_third unused/get_points_from_camera_third.cc)
target_link_libraries(unused_get_points_from_camera_third ${PROJECT_NAME})

add_executable(unused_points_seen_by_camera_by_frame_third unused/points_seen_by_camera_by_frame_third.cc)
target_link_libraries(unused_points_seen_by_camera_by_frame_third ${PROJECT_NAME})

add_executable(points_seen_by_frame_data points_seen_by_frame_data.cc)
target_link_libraries(points_seen_by_frame_data ${PROJECT_NAME})

add_executable(points_seen_by_frames points_seen_by_frames.cc)
target_link_libraries(points_seen_by_frames ${PROJECT_NAME})

add_executable(points_seen_by_pos points_seen_by_pos.cc)
target_link_libraries(points_seen_by_pos ${PROJECT_NAME})

#add_executable(mapping mapping.cc)
#target_link_libraries(mapping ${PROJECT_NAME})

#add_executable(simulator simulator.cc)
#target_link_libraries(simulator ${PROJECT_NAME})

add_executable(run_model run_model.cc)
target_link_libraries(run_model ${PROJECT_NAME})

#add_executable(get_pos_from_frame_orb_slam get_pos_from_frame_orb_slam.cc)
#target_link_libraries(get_pos_from_frame_orb_slam ${PROJECT_NAME})

#add_executable(check_matches check_matches.cc)
#target_link_libraries(check_matches ${PROJECT_NAME})

add_executable(icp_model_orbs_slam icp_model_orbs_slam.cc)
target_link_libraries(icp_model_orbs_slam ${PROJECT_NAME})

add_executable(take_frame_images take_frame_images.cc)
target_link_libraries(take_frame_images ${PROJECT_NAME})

#add_executable(run_orb_slam_matching run_orb_slam_matching.cc)
#target_link_libraries(run_orb_slam_matching ${PROJECT_NAME})

add_executable(check_matches_opencv check_matches_opencv.cc)
target_link_libraries(check_matches_opencv ${PROJECT_NAME})

add_executable(check_matches_one_by_one check_matches_one_by_one.cc)
target_link_libraries(check_matches_one_by_one ${PROJECT_NAME})

add_executable(run_model_with_printed_orbs run_model_with_printed_orbs.cc)
target_link_libraries(run_model_with_printed_orbs ${PROJECT_NAME})

add_executable(run_model_with_icp run_model_with_icp.cc)
target_link_libraries(run_model_with_icp ${PROJECT_NAME})

#add_executable(run_model_on_orb_slam run_model_on_orb_slam.cc)
#target_link_libraries(run_model_on_orb_slam ${PROJECT_NAME})

add_executable(see_keypoints see_keypoints.cc)
target_link_libraries(see_keypoints ${PROJECT_NAME})

add_executable(save_map save_map.cc)
target_link_libraries(save_map ${PROJECT_NAME})

add_executable(remove_outliers remove_outliers.cc)
target_link_libraries(remove_outliers ${PROJECT_NAME})

add_executable(run_model_with_orb_slam_points run_model_with_orb_slam_points.cc)
target_link_libraries(run_model_with_orb_slam_points ${PROJECT_NAME})

add_executable(pcd_to_xyz pcd_to_xyz.cc)
target_link_libraries(pcd_to_xyz ${PROJECT_NAME})

add_executable(csv_to_xyz csv_to_xyz.cc)
target_link_libraries(csv_to_xyz ${PROJECT_NAME})

add_executable(run_model_with_orbs_and_orb_slam_map run_model_with_orbs_and_orb_slam_map.cc)
target_link_libraries(run_model_with_orbs_and_orb_slam_map ${PROJECT_NAME})
add_executable(mono_tum mono_tum.cpp)
target_link_libraries(mono_tum ${PROJECT_NAME})
add_executable(test_exitRoomAlgo test_exitRoomAlgo.cpp)
target_link_libraries(test_exitRoomAlgo exitRoom)
