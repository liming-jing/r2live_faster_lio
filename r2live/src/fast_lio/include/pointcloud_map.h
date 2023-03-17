#ifndef _POINTCLOUD_MAP_H_
#define _POINTCLOUD_MAP_H_

#include "common_lib.h"
#include "kd_tree/ikd_Tree.h"


class PointCloudMap{
    public:
        PointCloudMap(double filter_size_map_min);
        void InitPointCloudMap(PointCloudXYZI::Ptr cloud);
        void DeletePointBoxes(std::vector<BoxPointType>& cub_needrm);
        void AddPointBoxes(std::vector<BoxPointType>& cub_needad);
        void AddPoints(PointCloudXYZI::Ptr cube_points_add);
        int GetPointsNumFromMap();


    private:
        KD_TREE ikdtree_;
        double filter_size_map_min_;
};

#endif