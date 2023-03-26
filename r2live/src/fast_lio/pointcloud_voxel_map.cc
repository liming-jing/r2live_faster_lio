#include "fast_lio/pointcloud_voxel_map.h"

PointCloudVoxelMap::PointCloudVoxelMap()
{
    Init();
}

void PointCloudVoxelMap::Init()
{
    
}

void PointCloudVoxelMap::TransformPointBody2World(const PointCloudXYZI::Ptr &point_cloud_body, pcl::PointCloud<pcl::PointXYZI>::Ptr &point_cloud_world)
{
    point_cloud_world->clear();

    for (size_t i = 0; i < point_cloud_body->size(); i++) {

        pcl::PointXYZINormal p_c = point_cloud_body->points[i];
        Eigen::Vector3d p(p_c.x, p_c.y, p_c.z);
        p = g_lio_state.rot_end * p + g_lio_state.pos_end;

        pcl::PointXYZI pi;
        pi.x = p(0);
        pi.y = p(1);
        pi.z = p(2);
        pi.intensity = p_c.intensity;

        point_cloud_world->points.push_back(pi);
    }
}