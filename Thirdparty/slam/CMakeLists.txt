include_directories(
        ${Pangolin_INCLUDE_DIRS}
)

add_library(ORB_SLAM2 STATIC
        src/System.cc
        src/Tracking.cc
        src/LocalMapping.cc
        src/ORBextractor.cc
        src/ORBmatcher.cc
        src/FrameDrawer.cc
        src/Sim3Solver.cc
        src/Converter.cc
        src/MapPoint.cc
        src/KeyFrame.cc
        src/Map.cc
        src/MapDrawer.cc
	src/CSVReader.cc
        src/Optimizer.cc
        src/PnPsolver.cc
        src/Frame.cc
        src/KeyFrameDatabase.cc
        src/Initializer.cc
        src/Viewer.cc
        src/LoopClosing.cc
)

target_link_libraries(ORB_SLAM2 PUBLIC
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        g2o
        DBoW3
        Threads::Threads
        Boost::boost
        Boost::serialization
        Boost::system
        ${PYTHON_LIBRARIES}
        nlohmann_json::nlohmann_json
)