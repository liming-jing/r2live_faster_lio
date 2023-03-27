#ifndef _POINTCLUOD_VOXEL_MAP_H_
#define _POINTCLUOD_VOXEL_MAP_H_

#include <vector>

#include "r2live/common_lib.h"
#include "voxel_map/voxel_map.h"
#include "pointcloud_map_base.h"

class PointCloudVoxelMap: public PointCloudMapBase {
    public:
        PointCloudVoxelMap();
        virtual void LaserMapFovSegment(Eigen::Vector3d pos) override {};
        virtual void InitPointCloudMap(PointCloudXYZI::Ptr cloud) override;
        virtual void NearestSearch(PointType& point, PointVector& point_near, int k_nearest, bool& point_selected_surf) override;
        virtual void AddNewPointCloud(PointCloudXYZI::Ptr cloud, std::vector<PointVector>& nearest_points, bool flg)  override;
    private:
        void Init();

        void TransformPointBody2World(const PointCloudXYZI::Ptr &point_cloud_body, pcl::PointCloud<pcl::PointXYZI>::Ptr &point_cloud_world);

        void CalcBodyCov(Eigen::Vector3d point, Eigen::Matrix3d& cov);
        void BuildVoxelMap(std::vector<pointWithCov>& pv_list);

    private:
        double max_voxel_size_;
        int max_layer_; 
        std::vector<int> layer_size_;
        int max_points_size_;
        double min_eigen_value_;
        double ranging_cov_;
        double angle_cov_;

        std::unordered_map<VOXEL_LOC, OctoTree *> voxel_map_;
};

#endif