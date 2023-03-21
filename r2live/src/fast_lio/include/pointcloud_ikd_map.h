#ifndef _POINTCLOUD_MAP_H_
#define _POINTCLOUD_MAP_H_

#include "pointcloud_map_base.h"
#include "kd_tree/ikd_Tree.h"

const int kLaserCloudWidth = 48;
const int kLaserCloudHeight = 48;
const int kLaserCloudDepth = 48;
const int kLaserCloudNum = kLaserCloudWidth * kLaserCloudHeight * kLaserCloudDepth;

const float kMovThreshold = 1.5f;
const float kDetRange = 300.0f;

class PointCloudIkdMap: public PointCloudMapBase {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    public:
        PointCloudIkdMap();
        virtual void InitPointCloudMap(PointCloudXYZI::Ptr cloud) override;
        virtual void LaserMapFovSegment(Eigen::Vector3d pos) override;
        virtual void NearestSearch(PointType & point, PointVector& points_near, int num, bool& point_selected_surf) override;
        virtual void AddNewPointCloud(PointCloudXYZI::Ptr cloud, std::vector<PointVector>& nearest_points, bool flg) override;

    private: 
        int DeletePointBoxes(std::vector<BoxPointType>& cub_needrm);
        void AddPointBoxes(std::vector<BoxPointType>& cub_needad);
        void AddPoints(PointCloudXYZI::Ptr cube_points_add);
        void AcquireRemovedPoints(PointVector& points_history);
        int GetPointsNumFromMap();
        void SetCenWidthHeightDepth(int width, int height, int depth);
        void PointTypeBodyToWorld(PointType const *const pi, PointType *const po);

        PointCloudXYZI::Ptr GetFeatureMap();
    private:
        void Init();
        
        template <typename T>
        void PointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi, Eigen::Matrix<T, 3, 1> &po)
        {
            Eigen::Vector3d p_body(pi[0], pi[1], pi[2]);
            Eigen::Vector3d p_global(g_lio_state.rot_end * (p_body + Lidar_offset_to_IMU) + g_lio_state.pos_end);
            po[0] = p_global(0);
            po[1] = p_global(1);
            po[2] = p_global(2);
        }

    private:
        int laser_cloud_cen_width_;
        int laser_cloud_cen_height_;
        int laser_cloud_cen_depth_;

        double filter_size_map_min_;
        bool cube_updated_[kLaserCloudNum];
        double cube_len_ = 0.0;
        double maximum_pt_kdtree_dis_ = 1.0;

        bool local_map_init_;
        std::vector<BoxPointType> cub_needrm_;
        Eigen::Vector3f x_axis_point_body_; //(LIDAR_SP_LEN, 0.0, 0.0);
        Eigen::Vector3f x_axis_point_world_; //(LIDAR_SP_LEN, 0.0, 0.0);
        BoxPointType local_map_points_;

    private:
        KD_TREE ikdtree_;
};

#endif