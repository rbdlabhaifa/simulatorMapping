add_subdirectory(DBoW3)
add_subdirectory(g2o)
add_subdirectory(Pangolin)

# Needs to find Pangolin for including in slam subdirectory
find_package(Pangolin REQUIRED PATHS "${CMAKE_CURRENT_SOURCE_DIR}/Pangolin")

add_subdirectory(slam)
