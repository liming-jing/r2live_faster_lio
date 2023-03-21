#ifndef _POINTCLOUD_IVOX_MAP_H_
#define _POINTCLOUD_IVOX_MAP_H_

#include <glog/logging.h>
#include "ivox3d/ivox3d.h"
#include "pointcloud_map_base.h"
#include <vector>

class PointCloudIvoxMap : public PointCloudMapBase {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#ifdef IVOX_NODE_TYPE_PHC
    using IVoxType = IVox<3, IVoxNodeType::PHC, PointType>;
#else
    using IVoxType = IVox<3, IVoxNodeType::DEFAULT, PointType>;
#endif

    public:
        PointCloudIvoxMap();
        virtual void LaserMapFovSegment(Eigen::Vector3d pos) override;
        virtual void InitPointCloudMap(PointCloudXYZI::Ptr cloud) override;
        virtual void NearestSearch(PointType& point, PointVector& point_near, int k_nearest, bool& point_selected_surf) override;
        virtual void AddNewPointCloud(PointCloudXYZI::Ptr cloud, std::vector<PointVector>& nearest_points, bool flg) override;
        int GetEffectPointsInMap();
    private:
        void Init();
        void PointTypeBodyToWorld(PointType const *const pi, PointType *const po);
    
    private:

        IVoxType::Options ivox_options_;
        std::shared_ptr<IVoxType> ivox_ = nullptr;                    // localmap in ivox
};

#endif