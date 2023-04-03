#include <memory>
#include <ros/ros.h>
#include <glog/logging.h>

#include "common_lib.h"

#include "voxel_mapping.h"

// Camera_Lidar_queue g_camera_lidar_queue;
MeasureGroup Measures;
StatesGroup g_lio_state;

int main(int argc, char **argv)
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = true;
    FLAGS_logtostderr = true;

    ros::init(argc, argv, "voxel_map_node");
    ros::NodeHandle nh;

    std::shared_ptr<VoxelMapping> voxel_map_ptr = std::make_shared<VoxelMapping>(nh);

    ros::spin();

    return 0;
}