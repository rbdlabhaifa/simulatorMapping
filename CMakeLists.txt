cmake_minimum_required(VERSION 2.8)
project(ORB_SLAM2)
find_package(nlohmann_json 3.2.0 REQUIRED)
IF (NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF ()
MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})
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
find_package(PythonLibs REQUIRED)
find_package(Threads REQUIRED)
find_package(Boost REQUIRED COMPONENTS system serialization python)
find_package(PythonLibs 3.10 REQUIRED)
message(PY_VERSION="${PY_VERSION}")

find_package(PCL 1.8 REQUIRED)
add_subdirectory(Thirdparty)
find_package(Pangolin REQUIRED PATHS "${CMAKE_CURRENT_SOURCE_DIR}Thirdparty/Pangolin")

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
        utils/src/Auxiliary.cpp
        utils/src/OfflineMapPoint.cpp
        utils/src/MapControl.cpp
        )


target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Boost_LIBRARIES}
        ${Pangolin_LIBRARIES}
        ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
        ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
        ${PCL_LIBRARIES}
        -lpthread ${PYTHON_LIBRARIES}
	    -lboost_serialization
	    -lboost_system
        )
add_library(simulator tools/simulator/simulator.cpp)
target_link_libraries(simulator ${PROJECT_NAME})
add_library(exitRoom tools/navigation/RoomExit.cpp)
target_link_libraries(exitRoom ${PROJECT_NAME})

add_library(simulator_python SHARED tools/simulator/simulator_python.cpp)

target_link_libraries(simulator_python boost_python310 ${PYTHON_LIBRARIES} ${Boost_LIBRARIES} simulator)

# Copy the __init__.py file
configure_file(tools/simulator/__init__.py ${CMAKE_CURRENT_BINARY_DIR}/__init__.py COPYONLY)

# Suppress prefix "lib" because Python does not allow this prefix
set_target_properties(simulator_python PROPERTIES PREFIX "")

message(PYTHON_INSTALL_PATH="${PYTHON_INSTALL_PATH}")

install(TARGETS simulator_python DESTINATION "${PYTHON_INSTALL_PATH}")

add_subdirectory(exe)
