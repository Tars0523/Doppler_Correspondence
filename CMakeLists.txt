cmake_minimum_required(VERSION 3.10)
project(Doppler_Correspondence LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ─── Dpendencies ──────────────────────────────────────────────────────────
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED COMPONENTS common kdtree)
find_package(catkin REQUIRED COMPONENTS
  roscpp sensor_msgs geometry_msgs nav_msgs)

# ─── Include paths ─────────────────────────────────────────────────────────
include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/core
  ${PROJECT_SOURCE_DIR}/pipeline
  ${PROJECT_SOURCE_DIR}/ros
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

# ─── Core library ─────────────────────────────────────────────────────────
add_library(doppler_core
  core/math_utils.cpp
  core/kdtree_util.cpp
  core/alignment.cpp)

target_link_libraries(doppler_core
  PUBLIC
    Eigen3::Eigen
    ${PCL_COMMON_LIBRARIES}
    ${PCL_KDTREE_LIBRARIES}
    OpenMP::OpenMP_CXX)

# ─── Pipeline library ─────────────────────────────────────────────────────
add_library(doppler_pipeline
  pipeline/correspondence.cpp
  pipeline/icp_iter.cpp
  pipeline/icp_doppler.cpp)

target_link_libraries(doppler_pipeline
  PUBLIC
    doppler_core
    ${PCL_COMMON_LIBRARIES}
    ${PCL_KDTREE_LIBRARIES}
    OpenMP::OpenMP_CXX)

# ─── Catkin_package  ──────────────────────────────────────────────
catkin_package(
  INCLUDE_DIRS core pipeline ros
  LIBRARIES doppler_core doppler_pipeline
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs nav_msgs
  DEPENDS Eigen3 PCL
)

# ─── ROS node executable ──────────────────────────────────────────────────
add_executable(icp_doppler_node
  ros/icp_doppler_.cpp
  ros/icp_doppler_node.cpp)

target_link_libraries(icp_doppler_node
  PRIVATE
    doppler_pipeline
    ${catkin_LIBRARIES}
    OpenMP::OpenMP_CXX)

# ─── Install ───────────────────────────────────────────────────
install(TARGETS icp_doppler_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS doppler_core doppler_pipeline
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY core pipeline ros
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
