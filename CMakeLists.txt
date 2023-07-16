cmake_minimum_required(VERSION 3.0)
project(ORB_SLAM2)
set(CMAKE_CXX_STANDARD 17)
IF (NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Debug)
ENDIF ()
MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -W0 -MT -bigobj")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -W0 -MT -bigobj")
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
list(APPEND VCPKG_FEATURE_FLAGS "versions")
find_package(lz4 CONFIG REQUIRED)
find_package(PythonLibs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PythonLibs REQUIRED)
find_package(Threads REQUIRED)
find_package(Boost REQUIRED COMPONENTS system serialization)
find_package(Pangolin REQUIRED PATHS "${CMAKE_CURRENT_SOURCE_DIR}Thirdparty/Pangolin")
find_package(OpenCV REQUIRED)
add_subdirectory(Thirdparty)
include_directories(
            ${EIGEN3_INCLUDE_DIR}        
        ${Pangolin_INCLUDE_DIRS}
        ${PYTHON_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/
        ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/
        ${PROJECT_SOURCE_DIR}/utils
        ${PROJECT_SOURCE_DIR}/slam/include/
        ${PROJECT_SOURCE_DIR}/tools
)

add_library(${PROJECT_NAME} STATIC
        slam/src/System.cc
        slam/src/Tracking.cc
        slam/src/LocalMapping.cc
        slam/src/ORBextractor.cc
        slam/src/ORBmatcher.cc
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


target_link_libraries(${PROJECT_NAME} PUBLIC
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        lz4::lz4
        ${Pangolin_LIBRARIES}
        g2o
        DBoW2
        Threads::Threads Boost::boost Boost::serialization Boost::system ${PYTHON_LIBRARIES} nlohmann_json::nlohmann_json
        )
add_library(simulator tools/simulator.cpp)
target_link_libraries(simulator ${PROJECT_NAME})
add_subdirectory(exe)