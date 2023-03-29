#ifndef _POINT_CLOUD_MAP_BASE_H_
#define _POINT_CLOUD_MAP_BASE_H_

#include <Eigen/Core>
#include "r2live/common_lib.h"
#include "r2live/parameter_server.h"
#include "voxel_map/voxel_map.h"

extern StatesGroup g_lio_state;

using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

class PointCloudMapBase {
    public:
        virtual void LaserMapFovSegment(Eigen::Vector3d pos) {}
        virtual void InitPointCloudMap(PointCloudXYZI::Ptr cloud) = 0;
        virtual void NearestSearch(PointType& point, PointVector& point_near, int k_nearest, bool& point_selected_surf) {}
        virtual void AddNewPointCloud(PointCloudXYZI::Ptr cloud, std::vector<PointVector>& nearest_points, bool flg) = 0;
        virtual void GetResidualList(const std::vector<pointWithCov>& pv_list, std::vector<ptpl> &ptpl_list, std::vector<Eigen::Vector3d> &non_match) {}
        virtual void SetParameters(std::vector<Eigen::Matrix3d>& body_var) {}
};

#endif