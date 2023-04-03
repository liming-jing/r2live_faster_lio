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


include_directories(
  ${catkin_INCLUDE_DIRS}
  ./src/fast_lio/include
  ./src/tools/
)


add_executable(voxel_map_node 

              src/apps/voxel_map_node.cc


              src/fast_lio/src/imu_processing.cc
              src/fast_lio/src/preprocess.cc
              # src/fast_lio/src/parameter_server.cc
              src/fast_lio/src/voxel_map_util.cc
              src/fast_lio/src/voxel_mapping.cc
)

target_link_libraries(voxel_map_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(r2live
    src/estimator_node.cpp
    src/parameters.cpp
    src/estimator.cpp
    src/estimator_optimization.cpp
    src/feature_manager.cpp
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
    
    # src/fast_lio/IMU_Processing.cpp

    src/fast_lio/src/imu_processing.cc
    src/fast_lio/src/preprocess.cc
    src/fast_lio/src/voxel_map_util.cc
    src/fast_lio/src/voxel_mapping.cc
    
    src/fast_lio/src/parameter_server.cc
    )

target_link_libraries(r2live ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES}) 





