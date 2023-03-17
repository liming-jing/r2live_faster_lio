#ifndef _POINTCLOUD_MAP_H_
#define _POINTCLOUD_MAP_H_

#include "common_lib.h"
#include "kd_tree/ikd_Tree.h"

const int kLaserCloudWidth = 48;
const int kLaserCloudHeight = 48;
const int kLaserCloudDepth = 48;
const int kLaserCloudNum = kLaserCloudWidth * kLaserCloudHeight * kLaserCloudDepth;

class PointCloudMap{
    public:
        PointCloudMap(double filter_size_map_min, double cube_len);
        void InitPointCloudMap(PointCloudXYZI::Ptr cloud);
        void DeletePointBoxes(std::vector<BoxPointType>& cub_needrm);
        void AddPointBoxes(std::vector<BoxPointType>& cub_needad);
        void AddPoints(PointCloudXYZI::Ptr cube_points_add);
        int GetPointsNumFromMap();
        void NearestSearch(PointType & point, int num, PointVector& points_near, std::vector<float>& point_search_sq_dis);

        void AddNewPointCloud(PointCloudXYZI::Ptr cloud, PointCloudXYZI::Ptr featsArray[]);

        void SetCenWidthHeightDepth(int width, int height, int depth);
    private:
        void Init();

    private:
        int laser_cloud_cen_width_;
        int laser_cloud_cen_height_;
        int laser_cloud_cen_depth_;

        double filter_size_map_min_;
        bool cube_updated_[kLaserCloudNum];
        double cube_len_ = 0.0;

        // PointCloudXYZI::Ptr featsArray_[kLaserCloudNum];

    private:
        KD_TREE ikdtree_;
};

#endif