cmake_minimum_required(VERSION 2.8)
project(ORB_SLAM2)
find_package(nlohmann_json 3.2.0 REQUIRED)
IF (NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF ()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -lncurses")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -lboost_serialization -lncurses")
#set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native")
#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -Wall   -O3 -lboost_serialization -march=native")

# Check C++17 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if (COMPILER_SUPPORTS_CXX17)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
    add_definitions(-DCOMPILEDWITHC17)
    message(STATUS "Using flag -std=c++17.")
elseif (COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    add_definitions(-DCOMPILEDWITHC0X)
    message(STATUS "Using flag -std=c++0x.")
else ()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++17 support. Please use a different C++ compiler.")
endif ()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 3.0 QUIET)
if (NOT OpenCV_FOUND)
    find_package(OpenCV 2.4.3 QUIET)
    if (NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
    endif ()
endif ()


find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)
find_package(Pangolin REQUIRED)
find_package(PythonLibs REQUIRED)
find_package(Threads REQUIRED)
find_package(Boost REQUIRED COMPONENTS system serialization)
find_package(PCL 1.8 REQUIRED)

include_directories(

        ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/
        ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/
        ${PROJECT_SOURCE_DIR}/Thirdparty/Pangolin/
        ${PROJECT_SOURCE_DIR}/Thirdparty/eigen-3.1.3/
        ${PROJECT_SOURCE_DIR}/utils
        ${PROJECT_SOURCE_DIR}/slam/include
        ${EIGEN3_INCLUDE_DIR}
        ${Pangolin_INCLUDE_DIRS}
        ${PYTHON_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/tools
)

# Add definitions
add_definitions(${PCL_DEFINITIONS})

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME}
        slam/src/System.cc
        slam/src/Tracking.cc
        slam/src/LocalMapping.cc
        slam/src/ORBextractor.cc
        slam/src/ORBmatcher.cc
        slam/src/RaspberryKeyFrame.cc
        slam/src/FrameDrawer.cc
        slam/src/Sim3Solver.cc
        slam/src/Converter.cc
        slam/src/MapPoint.cc
        slam/src/KeyFrame.cc
        slam/src/Map.cc
        slam/src/MapDrawer.cc
	    slam/src/CSVReader.cc
        slam/src/Optimizer.cc
        slam/src/PnPsolver.cc
        slam/src/Frame.cc
        slam/src/KeyFrameDatabase.cc
        slam/src/Initializer.cc
        slam/src/Viewer.cc
        slam/src/LoopClosing.cc
        utils/src/Point.cpp
        utils/src/Line.cpp
        utils/src/Auxiliary.cpp
        )


target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Boost_LIBRARIES}
        ${Pangolin_LIBRARIES}
        ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
        ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
        ${PCL_LIBRARIES}
        -lpthread ctello spdlog ${PYTHON_LIBRARIES} -lbluetooth
	    -lboost_serialization
	    -lboost_system
        )
add_library(simulator tools/simulator.cpp)
target_link_libraries(simulator ${PROJECT_NAME})
add_executable(runSimulator exe/runSimulator.cpp)
target_link_libraries(runSimulator simulator)

add_executable(offline_orb_slam exe/offline_orb_slam.cc)
target_link_libraries(offline_orb_slam ${PROJECT_NAME})

add_executable(get_point_data exe/get_point_data.cc)
target_link_libraries(get_point_data ${PROJECT_NAME})

add_executable(unused_get_points_from_camera exe/unused/get_points_from_camera.cc)
target_link_libraries(unused_get_points_from_camera ${PROJECT_NAME})

add_executable(unused_test_is_point_visible exe/unused/test_is_point_visible.cc)
target_link_libraries(unused_test_is_point_visible ${PROJECT_NAME})

add_executable(unused_get_points_from_camera_second exe/unused/get_points_from_camera_second.cc)
target_link_libraries(unused_get_points_from_camera_second ${PROJECT_NAME})

add_executable(unused_points_seen_by_camera_by_frame exe/unused/points_seen_by_camera_by_frame.cc)
target_link_libraries(unused_points_seen_by_camera_by_frame ${PROJECT_NAME})

add_executable(unused_points_seen_by_camera_by_frame_second exe/unused/points_seen_by_camera_by_frame_second.cc)
target_link_libraries(unused_points_seen_by_camera_by_frame_second ${PROJECT_NAME})

add_executable(get_points_from_frame_orb_slam exe/get_points_from_frame_orb_slam.cc)
target_link_libraries(get_points_from_frame_orb_slam ${PROJECT_NAME})

add_executable(unused_get_points_from_camera_third exe/unused/get_points_from_camera_third.cc)
target_link_libraries(unused_get_points_from_camera_third ${PROJECT_NAME})

add_executable(unused_points_seen_by_camera_by_frame_third exe/unused/points_seen_by_camera_by_frame_third.cc)
target_link_libraries(unused_points_seen_by_camera_by_frame_third ${PROJECT_NAME})

add_executable(points_seen_by_frame_data exe/points_seen_by_frame_data.cc)
target_link_libraries(points_seen_by_frame_data ${PROJECT_NAME})

add_executable(points_seen_by_frames exe/points_seen_by_frames.cc)
target_link_libraries(points_seen_by_frames ${PROJECT_NAME})

add_executable(points_seen_by_pos exe/points_seen_by_pos.cc)
target_link_libraries(points_seen_by_pos ${PROJECT_NAME})

add_executable(mapping exe/mapping.cc)
target_link_libraries(mapping ${PROJECT_NAME})

#add_executable(simulator exe/simulator.cc)
#target_link_libraries(simulator ${PROJECT_NAME})

add_executable(run_model exe/run_model.cc)
target_link_libraries(run_model ${PROJECT_NAME})

add_executable(get_pos_from_frame_orb_slam exe/get_pos_from_frame_orb_slam.cc)
target_link_libraries(get_pos_from_frame_orb_slam ${PROJECT_NAME})

add_executable(check_matches exe/check_matches.cc)
target_link_libraries(check_matches ${PROJECT_NAME})

add_executable(icp_model_orbs_slam exe/icp_model_orbs_slam.cc)
target_link_libraries(icp_model_orbs_slam ${PROJECT_NAME})

add_executable(take_frame_images exe/take_frame_images.cc)
target_link_libraries(take_frame_images ${PROJECT_NAME})

add_executable(run_orb_slam_matching exe/run_orb_slam_matching.cc)
target_link_libraries(run_orb_slam_matching ${PROJECT_NAME})

add_executable(check_matches_opencv exe/check_matches_opencv.cc)
target_link_libraries(check_matches_opencv ${PROJECT_NAME})

add_executable(check_matches_one_by_one exe/check_matches_one_by_one.cc)
target_link_libraries(check_matches_one_by_one ${PROJECT_NAME})

add_executable(run_model_with_printed_orbs exe/run_model_with_printed_orbs.cc)
target_link_libraries(run_model_with_printed_orbs ${PROJECT_NAME})

add_executable(run_model_with_icp exe/run_model_with_icp.cc)
target_link_libraries(run_model_with_icp ${PROJECT_NAME})

add_executable(run_model_on_orb_slam exe/run_model_on_orb_slam.cc)
target_link_libraries(run_model_on_orb_slam ${PROJECT_NAME})

add_executable(see_keypoints exe/see_keypoints.cc)
target_link_libraries(see_keypoints ${PROJECT_NAME})

add_executable(save_map exe/save_map.cc)
target_link_libraries(save_map ${PROJECT_NAME})

add_executable(remove_outliers exe/remove_outliers.cc)
target_link_libraries(remove_outliers ${PROJECT_NAME})

add_executable(run_model_with_orb_slam_points exe/run_model_with_orb_slam_points.cc)
target_link_libraries(run_model_with_orb_slam_points ${PROJECT_NAME})
