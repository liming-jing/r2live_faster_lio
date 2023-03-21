#include "pointcloud_ikd_map.h"


PointCloudMap::PointCloudMap() 
{
    Init();
}

void PointCloudMap::Init()
{
    ParameterServer* para_server = ParameterServer::GetInstance();
    filter_size_map_min_ = para_server->GetFilterSizeMapMin();
    cube_len_ = para_server->GetCubeLen();
    maximum_pt_kdtree_dis_ = para_server->GetMaximumPtKdtreeDis();

     local_map_init_ = false;
         x_axis_point_body_ = Eigen::Vector3f(LIDAR_SP_LEN, 0.0, 0.0);
    x_axis_point_world_ = Eigen::Vector3f(LIDAR_SP_LEN, 0.0, 0.0);
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

int PointCloudMap::DeletePointBoxes(std::vector<BoxPointType>& cub_needrm)
{
    int kdtree_delete_counter = 0;
    if (ikdtree_.Root_Node != nullptr)
    {
        kdtree_delete_counter = ikdtree_.Delete_Point_Boxes(cub_needrm);
    }
    else {
        std::cerr << "Ikdtree not initialized" << std::endl;
    }
    return kdtree_delete_counter;
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

void PointCloudMap::AcquireRemovedPoints(PointVector& points_history)
{
    if (ikdtree_.Root_Node != nullptr)
    {
        ikdtree_.acquire_removed_points(points_history);
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

void PointCloudMap::NearestSearch(PointType & point, PointVector& points_near, int k_nearest, bool& point_selected_surf)
{
     if (ikdtree_.Root_Node != nullptr)
    {
        std::vector<float> pointSearchSqDis_surf;
        ikdtree_.Nearest_Search(point, k_nearest, points_near, pointSearchSqDis_surf);

        float max_distance = pointSearchSqDis_surf.back();
        
        if (max_distance > maximum_pt_kdtree_dis_)
        {
            point_selected_surf = false;
        }
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

void PointCloudMap::AddNewPointCloud(PointCloudXYZI::Ptr cloud, std::vector<PointVector>& nearest_points, bool flg)
{
    PointVector points_history;
    ikdtree_.acquire_removed_points(points_history);

    // memset(cube_updated_, 0, sizeof(cube_updated_));

    // for (int i = 0; i < points_history.size(); i++)
    // {
    //     PointType &pointSel = points_history[i];

    //     int cubeI = int((pointSel.x + 0.5 * cube_len_) / cube_len_) + laser_cloud_cen_width_;
    //     int cubeJ = int((pointSel.y + 0.5 * cube_len_) / cube_len_) + laser_cloud_cen_height_;
    //     int cubeK = int((pointSel.z + 0.5 * cube_len_) / cube_len_) + laser_cloud_cen_depth_;

    //     if (pointSel.x + 0.5 * cube_len_ < 0)
    //         cubeI--;
    //     if (pointSel.y + 0.5 * cube_len_ < 0)
    //         cubeJ--;
    //     if (pointSel.z + 0.5 * cube_len_ < 0)
    //         cubeK--;

    //     if (cubeI >= 0 && cubeI < laser_cloud_cen_width_ &&
    //         cubeJ >= 0 && cubeJ < laser_cloud_cen_height_ &&
    //         cubeK >= 0 && cubeK < laser_cloud_cen_depth_)
    //     {
    //         int cubeInd = cubeI + laser_cloud_cen_width_ * cubeJ + laser_cloud_cen_width_ * laser_cloud_cen_height_ * cubeK;
    //         featsArray[cubeInd]->push_back(pointSel);
    //     }
    // }

    int points_size = cloud->points.size();
    PointCloudXYZI::Ptr feats_down_updated(new PointCloudXYZI(*cloud));
    for (int i = 0; i < points_size; i++)
    {
        PointTypeBodyToWorld(&(cloud->points[i]), &(feats_down_updated->points[i]));
    }

    ikdtree_.Add_Points(feats_down_updated->points, true);
}

void PointCloudMap::LaserMapFovSegment(Eigen::Vector3d pos)
{
    cub_needrm_.clear();
    static float Mov_DetRange_result = kMovThreshold * kDetRange;
    PointBodyToWorld(x_axis_point_body_, x_axis_point_world_);
    Eigen::Vector3d pos_LiD = pos;
    if (!local_map_init_){
        for (int i = 0; i < 3; i++){
            local_map_points_.vertex_min[i] = pos_LiD(i) - cube_len_ / 2.0;
            local_map_points_.vertex_max[i] = pos_LiD(i) + cube_len_ / 2.0;
        }
        local_map_init_ = true;
        return;
    }

    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - local_map_points_.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - local_map_points_.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= Mov_DetRange_result || dist_to_map_edge[i][1] <= Mov_DetRange_result) 
        {
            need_move = true;
        }
    }

    if (!need_move) return;

    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = local_map_points_;
    float mov_dist = max((cube_len_ - 2.0 * Mov_DetRange_result) * 0.5 * 0.9, double(kDetRange * (kMovThreshold -1)));
    for (int i = 0; i < 3; i++){
        tmp_boxpoints = local_map_points_;
        if (dist_to_map_edge[i][0] <= Mov_DetRange_result){
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = local_map_points_.vertex_max[i] - mov_dist;
            cub_needrm_.push_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= Mov_DetRange_result){
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = local_map_points_.vertex_min[i] + mov_dist;
            cub_needrm_.push_back(tmp_boxpoints);
        }
    }
    local_map_points_ = New_LocalMap_Points;

    PointVector points_history;
    AcquireRemovedPoints(points_history);

    if(cub_needrm_.size() > 0) 
    {
        DeletePointBoxes(cub_needrm_);
    }
}

PointCloudXYZI::Ptr PointCloudMap::GetFeatureMap()
{
    PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
    PointVector().swap(ikdtree_.PCL_Storage);
    ikdtree_.flatten(ikdtree_.Root_Node, ikdtree_.PCL_Storage, NOT_RECORD);
    featsFromMap->clear();
    featsFromMap->points = ikdtree_.PCL_Storage;
    return featsFromMap;
}

void PointCloudMap::PointTypeBodyToWorld(PointType const *const pi, PointType *const po)
{
    Eigen::Vector3d p_body(pi->x, pi->y, pi->z);
    Eigen::Vector3d p_global(g_lio_state.rot_end * (p_body + Lidar_offset_to_IMU) + g_lio_state.pos_end);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}
