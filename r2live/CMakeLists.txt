cmake_minimum_required(VERSION 2.8.3)
project(r2live)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -std=c++17 -O3 -w")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    tf
    cv_bridge
    message_generation
    livox_ros_driver
    pcl_ros
    pcl_conversions
    )

add_message_files(
      FILES
      Pose6D.msg
      States.msg
      cloud_info.msg
    )
    
generate_messages(
     DEPENDENCIES
     geometry_msgs
    )
    
catkin_package(DEPENDS PCL)

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(ALL_TARGET_LIBRARIES "")

include(cmake/eigen.cmake)
include(cmake/glog.cmake)
include(cmake/OpenCV.cmake)
include(cmake/OpenMP.cmake)
include(cmake/pcl.cmake)
include(cmake/yaml.cmake)
include(cmake/ceres.cmake)
include(cmake/tbb.cmake)


include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(lio_feat_extract 
              apps/lio_feature_extract_node.cc
              src/fast_lio/feature_extract.cc
              src/r2live/parameter_server.cc
)
target_link_libraries(lio_feat_extract ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(r2live
    apps/r2live_node.cc
    
    src/vins/parameters.cpp
    src/vins/estimator.cpp
    src/vins/estimator_optimization.cpp
    src/vins/feature_manager.cpp

    src/factor/pose_local_parameterization.cpp
    src/factor/projection_factor.cpp
    src/factor/projection_td_factor.cpp

    src/utility/utility.cpp
    src/utility/visualization.cpp
    src/utility/CameraPoseVisualization.cpp

    src/initial/solve_5pts.cpp
    src/initial/initial_aligment.cpp
    src/initial/initial_sfm.cpp
    src/initial/initial_ex_rotation.cpp
    
    src/fast_lio/fast_lio.cc 
    src/fast_lio/lio_core.cc 
    src/r2live/parameter_server.cc
    src/fast_lio/imu_process.cc
    src/fast_lio/pointcloud_ivox_map.cc
    src/fast_lio/pointcloud_ikd_map.cc 
    src/fast_lio/pointcloud_voxel_map.cc
    include/fast_lio/kd_tree/ikd_Tree.cpp
    include/fast_lio/voxel_map/voxel_map.cc
    
    src/r2live/r2live.cc
    )

target_link_libraries(r2live ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES}) 





