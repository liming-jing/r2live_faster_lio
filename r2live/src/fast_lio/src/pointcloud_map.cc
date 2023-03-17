#include "pointcloud_map.h"


PointCloudMap::PointCloudMap(double filter_size_map_min) 
{
    filter_size_map_min_ = filter_size_map_min;
}

void PointCloudMap::InitPointCloudMap(PointCloudXYZI::Ptr cloud)
{
    if (ikdtree_.Root_Node == nullptr)
    {
        ikdtree_.set_downsample_param(filter_size_map_min_);
        ikdtree_.Build(cloud->points);
    }
    std::cout << "Ikdtree initialization succeeded" << std::endl;
}

void PointCloudMap::DeletePointBoxes(std::vector<BoxPointType>& cub_needrm)
{
    if (ikdtree_.Root_Node != nullptr)
    {
        ikdtree_.Delete_Point_Boxes(cub_needrm);
    }
    else {
        std::cerr << "Ikdtree not initialized" << std::endl;
    }
}
void PointCloudMap::AddPointBoxes(std::vector<BoxPointType>& cub_needad)
{
    if (ikdtree_.Root_Node != nullptr)
    {
        ikdtree_.Add_Point_Boxes(cub_needad);
    }
    else {
        std::cerr << "Ikdtree not initialized" << std::endl;
    }
}
void PointCloudMap::AddPoints(PointCloudXYZI::Ptr cube_points_add)
{
    if (ikdtree_.Root_Node != nullptr)
    {
        ikdtree_.Add_Points(cube_points_add->points, true);
    }
    else {
        std::cerr << "Ikdtree not initialized" << std::endl;
    }
}

int PointCloudMap::GetPointsNumFromMap()
{
    if (ikdtree_.Root_Node != nullptr)
    {
        return ikdtree_.size();
    }
    else {
        std::cerr << "Ikdtree not initialized" << std::endl;
        return 0;
    }
}
