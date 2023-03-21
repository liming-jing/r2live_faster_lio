#ifndef _POINT_CLOUD_MAP_BASE_H_
#define _POINT_CLOUD_MAP_BASE_H_

#include <Eigen/Core>
#include "common_lib.h"
#include "parameter_server.h"

extern StatesGroup g_lio_state;

using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

class PointCloudMapBase {
    public:
        virtual void LaserMapFovSegment(Eigen::Vector3d pos) = 0;
        virtual void InitPointCloudMap(PointCloudXYZI::Ptr cloud) = 0;
        virtual void NearestSearch(PointType& point, PointVector& point_near, int k_nearest, bool& point_selected_surf) = 0;
        virtual void AddNewPointCloud(PointCloudXYZI::Ptr cloud, std::vector<PointVector>& nearest_points, bool flg) = 0;
    private:
        inline float calc_dist(const PointType &p1, const PointType &p2) 
        {
            return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
        }

        inline float calc_dist(const Eigen::Vector3f &p1, const Eigen::Vector3f &p2) { return (p1 - p2).squaredNorm(); }
};

#endif