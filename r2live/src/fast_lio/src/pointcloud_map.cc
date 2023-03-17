#include "pointcloud_map.h"


PointCloudMap::PointCloudMap(double filter_size_map_min, double cube_len) 
{
    // Init();
    filter_size_map_min_ = filter_size_map_min;
    cube_len_ = cube_len;
}

void PointCloudMap::Init()
{
    // for (int i = 0; i < kLaserCloudNum; i++)
    // {
    //     featsArray_[i].reset(new PointCloudXYZI());
    // }
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

void PointCloudMap::NearestSearch(PointType & point, int k_nearest, PointVector& points_near, std::vector<float>& point_search_sq_dis)
{
     if (ikdtree_.Root_Node != nullptr)
    {
       ikdtree_.Nearest_Search(point, k_nearest, points_near, point_search_sq_dis);
    }
    else {
        std::cerr << "Ikdtree not initialized" << std::endl;
    }
}

void PointCloudMap::SetCenWidthHeightDepth(int width, int height, int depth)
{
    laser_cloud_cen_width_ = width;
    laser_cloud_cen_height_ = height;
    laser_cloud_cen_depth_ = depth;
}

void PointCloudMap::AddNewPointCloud(PointCloudXYZI::Ptr cloud, PointCloudXYZI::Ptr featsArray[])
{
    PointVector points_history;
    ikdtree_.acquire_removed_points(points_history);

    memset(cube_updated_, 0, sizeof(cube_updated_));

    for (int i = 0; i < points_history.size(); i++)
    {
        PointType &pointSel = points_history[i];

        int cubeI = int((pointSel.x + 0.5 * cube_len_) / cube_len_) + laser_cloud_cen_width_;
        int cubeJ = int((pointSel.y + 0.5 * cube_len_) / cube_len_) + laser_cloud_cen_height_;
        int cubeK = int((pointSel.z + 0.5 * cube_len_) / cube_len_) + laser_cloud_cen_depth_;

        if (pointSel.x + 0.5 * cube_len_ < 0)
            cubeI--;
        if (pointSel.y + 0.5 * cube_len_ < 0)
            cubeJ--;
        if (pointSel.z + 0.5 * cube_len_ < 0)
            cubeK--;

        if (cubeI >= 0 && cubeI < laser_cloud_cen_width_ &&
            cubeJ >= 0 && cubeJ < laser_cloud_cen_height_ &&
            cubeK >= 0 && cubeK < laser_cloud_cen_depth_)
        {
            int cubeInd = cubeI + laser_cloud_cen_width_ * cubeJ + laser_cloud_cen_width_ * laser_cloud_cen_height_ * cubeK;
            featsArray[cubeInd]->push_back(pointSel);
        }
    }
    ikdtree_.Add_Points(cloud->points, true);
}
