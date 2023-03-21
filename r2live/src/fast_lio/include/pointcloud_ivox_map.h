#ifndef _POINTCLOUD_IVOX_MAP_H_
#define _POINTCLOUD_IVOX_MAP_H_

#include <glog/logging.h>
#include <Eigen/Core>
#include "ivox3d/ivox3d.h"
#include "common_lib.h"
#include "parameter_server.h"
#include <vector>

extern StatesGroup g_lio_state;

class PointCloudIvoxMap {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

#ifdef IVOX_NODE_TYPE_PHC
    using IVoxType = IVox<3, IVoxNodeType::PHC, PointType>;
#else
    using IVoxType = IVox<3, IVoxNodeType::DEFAULT, PointType>;
#endif

    using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

    public:
        PointCloudIvoxMap();
        void Init();
        void LaserMapFovSegment(Eigen::Vector3d pos) {}
        void InitPointCloudMap(PointCloudXYZI::Ptr cloud);
        void NearestSearch(PointType& point, PointVector& point_near, int k_nearest);
        void AddNewPointCloud(PointCloudXYZI::Ptr cloud, std::vector<PointVector>& nearest_points, bool flg);
    private:
        void PointTypeBodyToWorld(PointType const *const pi, PointType *const po);

        inline float calc_dist(const PointType &p1, const PointType &p2) {
            return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
        }

        inline float calc_dist(const Eigen::Vector3f &p1, const Eigen::Vector3f &p2) { return (p1 - p2).squaredNorm(); }
    
    private:

        IVoxType::Options ivox_options_;
        std::shared_ptr<IVoxType> ivox_ = nullptr;                    // localmap in ivox
};

#endif