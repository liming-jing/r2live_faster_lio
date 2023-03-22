#include "fast_lio/pointcloud_ivox_map.h"

PointCloudIvoxMap::PointCloudIvoxMap()
{
    Init();
}

void PointCloudIvoxMap::Init()
{
    ParameterServer* para_server = ParameterServer::GetInstance();
    ivox_options_.resolution_ = para_server->GetIvoxGridResolution();

    switch(para_server->GetIvoxNearbyType()) {
        case 0: 
            ivox_options_.nearby_type_ = IVoxType::NearbyType::CENTER;
            break;
        case 6:
            ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY6;
            break;
        case 18:
            ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
            break;
        case 26:
            ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY26;
            break;
        default:
             LOG(WARNING) << "unknown ivox_nearby_type, use NEARBY18";
             ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    }
}

 void PointCloudIvoxMap::LaserMapFovSegment(Eigen::Vector3d pos) {}

void PointCloudIvoxMap::InitPointCloudMap(PointCloudXYZI::Ptr cloud)
{
    ivox_ = std::make_shared<IVoxType>(ivox_options_);
    ivox_->AddPoints(cloud->points);
}

void PointCloudIvoxMap::NearestSearch(PointType& point, PointVector& points_near, int k_nearest, bool& point_selected_surf)
{
    ivox_->GetClosestPoint(point, points_near, k_nearest);
    
    point_selected_surf = points_near.size() >= k_nearest;
}

void PointCloudIvoxMap::AddNewPointCloud(PointCloudXYZI::Ptr cloud, std::vector<PointVector>& nearest_points, bool flg)
{
    PointVector points_to_add;
    PointVector point_no_need_downsample;

    int cur_pts = cloud->size();
    points_to_add.reserve(cur_pts);
    point_no_need_downsample.reserve(cur_pts);

    std::vector<size_t> index(cur_pts);
    for (size_t i = 0; i < cur_pts; ++i) {
        index[i] = i;
    }

    PointCloudXYZI::Ptr cloud_world(new PointCloudXYZI(*cloud));

    // bool flg = ParameterServer::GetInstance()->GetFlgEKFInited();
    double filter_size_map_min = ParameterServer::GetInstance()->GetFilterSizeMapMin();
    int num_match_points = ParameterServer::GetInstance()->GetNumMatchPoints();

    for (int i = 0; i < cur_pts; i++)
    {
        PointTypeBodyToWorld(&(cloud->points[i]), &(cloud_world->points[i]));

        /* decide if need add to map */
        PointType &point_world = cloud_world->points[i];
        if (!nearest_points[i].empty() && flg) {
            const PointVector &points_near = nearest_points[i];

            Eigen::Vector3f center =
                ((point_world.getVector3fMap() / filter_size_map_min).array().floor() + 0.5) * filter_size_map_min;

            Eigen::Vector3f dis_2_center = points_near[0].getVector3fMap() - center;

            if (fabs(dis_2_center.x()) > 0.5 * filter_size_map_min &&
                fabs(dis_2_center.y()) > 0.5 * filter_size_map_min &&
                fabs(dis_2_center.z()) > 0.5 * filter_size_map_min) {
                point_no_need_downsample.emplace_back(point_world);
                return;
            }

            bool need_add = true;
            float dist = calc_dist(point_world.getVector3fMap(), center);
            if (points_near.size() >= num_match_points) {
                for (int readd_i = 0; readd_i < num_match_points; readd_i++) {
                    if (calc_dist(points_near[readd_i].getVector3fMap(), center) < dist + 1e-6) {
                        need_add = false;
                        break;
                    }
                }
            }
            if (need_add) {
                points_to_add.emplace_back(point_world);
            }
        } else {
            points_to_add.emplace_back(point_world);
        }
    }

    ivox_->AddPoints(points_to_add);
    ivox_->AddPoints(point_no_need_downsample);
}

void PointCloudIvoxMap::PointTypeBodyToWorld(PointType const *const pi, PointType *const po)
{
    Eigen::Vector3d p_body(pi->x, pi->y, pi->z);
    Eigen::Vector3d p_global(g_lio_state.rot_end * (p_body + Lidar_offset_to_IMU) + g_lio_state.pos_end);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}
