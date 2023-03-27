#include "fast_lio/pointcloud_voxel_map.h"

PointCloudVoxelMap::PointCloudVoxelMap()
{
    Init();
}

void PointCloudVoxelMap::Init()
{
    ParameterServer* para_server = ParameterServer::GetInstance();
    max_voxel_size_ = para_server->GetMaxVoxelSize();
    max_layer_ = para_server->GetMaxLayer();
    max_points_size_ = para_server->GetMaxPointsSize();
    min_eigen_value_ = para_server->GetMinEigenValue();
    layer_size_ = para_server->GetLayerPointSize();

    ranging_cov_ = para_server->GetRangingCov();
    angle_cov_ = para_server->GetAngleCov();
}

void PointCloudVoxelMap::InitPointCloudMap(PointCloudXYZI::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr world_lidar(new pcl::PointCloud<pcl::PointXYZI>);
    TransformPointBody2World(cloud, world_lidar);

    std::vector<pointWithCov> pv_list;
    for (size_t i = 0; i < world_lidar->size(); i++) 
    {
        pointWithCov pv;
        pv.point << world_lidar->points[i].x, world_lidar->points[i].y,
            world_lidar->points[i].z;
        Eigen::Vector3d point_this(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);

        // if z=0, error will occur in calcBodyCov. To be solved
        if (point_this[2] == 0) 
        {
            point_this[2] = 0.001;
        }

        Eigen::Matrix3d cov;
        CalcBodyCov(point_this, cov);

        point_this += Lidar_offset_to_IMU;
        Eigen::Matrix3d point_crossmat;
        point_crossmat << SKEW_SYM_MATRX(point_this);

        cov = g_lio_state.rot_end * cov * g_lio_state.rot_end.transpose() +
            (-point_crossmat) * g_lio_state.cov.block<3, 3>(0, 0) *
                (-point_crossmat).transpose() +
            g_lio_state.cov.block<3, 3>(3, 3);
        pv.cov = cov;

        pv_list.push_back(pv);
    }

    BuildVoxelMap(pv_list);
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

void PointCloudVoxelMap::CalcBodyCov(Eigen::Vector3d point, Eigen::Matrix3d& cov)
{
    float range = sqrt(point[0] * point[0] + point[1] * point[1] + point[2] * point[2]);
    static float range_var = ranging_cov_ * ranging_cov_;

    Eigen::Matrix2d direction_var;
    direction_var << pow(sin(DEG2RAD(angle_cov_)), 2), 0, 0, pow(sin(DEG2RAD(angle_cov_)), 2);
    
    Eigen::Vector3d direction(point);
    direction.normalize();
    Eigen::Matrix3d direction_hat;
    direction_hat << 0, -direction(2), direction(1), direction(2), 0,
        -direction(0), -direction(1), direction(0), 0;

    Eigen::Vector3d base_vector1(1, 1,  -(direction(0) + direction(1)) / direction(2));
    base_vector1.normalize();

    Eigen::Vector3d base_vector2 = base_vector1.cross(direction);
    base_vector2.normalize();

    Eigen::Matrix<double, 3, 2> N;
    N <<    base_vector1(0), base_vector2(0), 
            base_vector1(1), base_vector2(1),
            base_vector1(2), base_vector2(2);

    Eigen::Matrix<double, 3, 2> A = range * direction_hat * N;

    cov = direction * range_var * direction.transpose() +
            A * direction_var * A.transpose();
}

void PointCloudVoxelMap::BuildVoxelMap(std::vector<pointWithCov>& pv_list)
{
    uint plsize = pv_list.size();
    for (uint i = 0; i < plsize; i++) 
    {
        const pointWithCov p_v = pv_list[i];
        float loc_xyz[3];
        for (int j = 0; j < 3; j++) 
        {
            loc_xyz[j] = p_v.point[j] / max_voxel_size_;
            if (loc_xyz[j] < 0) 
            {
                loc_xyz[j] -= 1.0;
            }
        }

        VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);

        auto iter = voxel_map_.find(position);
        if (iter != voxel_map_.end()) 
        {
            voxel_map_[position]->temp_points_.push_back(p_v);
            voxel_map_[position]->new_points_num_++;
        } 
        else 
        {
            OctoTree *octo_tree =
                new OctoTree(max_layer_, 0, layer_size_, max_points_size_,
                            max_points_size_, min_eigen_value_);
            voxel_map_[position] = octo_tree;
            voxel_map_[position]->quater_length_ = max_voxel_size_ / 4;
            voxel_map_[position]->voxel_center_[0] = (0.5 + position.x) * max_voxel_size_;
            voxel_map_[position]->voxel_center_[1] = (0.5 + position.y) * max_voxel_size_;
            voxel_map_[position]->voxel_center_[2] = (0.5 + position.z) * max_voxel_size_;
            voxel_map_[position]->temp_points_.push_back(p_v);
            voxel_map_[position]->new_points_num_++;
            voxel_map_[position]->layer_point_size_ = layer_size_;
        }
    }

    for (auto iter = voxel_map_.begin(); iter != voxel_map_.end(); ++iter) 
    {
        iter->second->init_octo_tree();
    }
}

