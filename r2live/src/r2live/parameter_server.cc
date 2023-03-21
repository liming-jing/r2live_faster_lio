#include "r2live/parameter_server.h"

ParameterServer* ParameterServer::instance = nullptr;

void ParameterServer::InitParamWithRos(ros::NodeHandle & nh)
{
    /*fast_lio parameters*/
    GetROSParameter(nh, "imu_topic", imu_topic_, std::string("/livox/imu"));
    GetROSParameter(nh, "fast_lio/dense_map_enable", dense_map_en_, true);
    GetROSParameter(nh, "fast_lio/lidar_time_delay", lidar_time_delay_, 0.0);
    GetROSParameter(nh, "fast_lio/max_iteration", num_max_iterations_, 4);
    GetROSParameter(nh, "fast_lio/fov_degree", fov_deg_, 70.00);
    GetROSParameter(nh, "fast_lio/filter_size_corner", filter_size_corner_min_, 0.4);
    GetROSParameter(nh, "fast_lio/filter_size_surf", filter_size_surf_min_, 0.4);
    GetROSParameter(nh, "fast_lio/filter_size_surf_z", filter_size_surf_min_z_, 0.4);
    GetROSParameter(nh, "fast_lio/filter_size_map", filter_size_map_min_, 0.4);
    GetROSParameter(nh, "fast_lio/cube_side_length", cube_len_, 100.0);
    GetROSParameter(nh, "fast_lio/maximum_pt_kdtree_dis", maximum_pt_kdtree_dis_, 3.0);
    GetROSParameter(nh, "fast_lio/maximum_res_dis", maximum_res_dis_, 3.0);
    GetROSParameter(nh, "fast_lio/planar_check_dis", planar_check_dis_, 0.05);
    GetROSParameter(nh, "fast_lio/long_rang_pt_dis", long_rang_pt_dis_, 50.0);
    GetROSParameter(nh, "fast_lio/publish_feature_map", if_publish_feature_map_, false);

    GetROSParameter(nh, "fast_lio/map_method", map_method_, std::string("ivox"));

    /* feature parameters */
    GetROSParameter(nh,"feature_extraction/N_SCANS", n_scans_, 1800);

    GetROSParameter(nh, "fast_lio/ivox_grid_resolution", ivox_grid_resolution_, 0.5);
    GetROSParameter(nh, "fast_lio/ivox_nearby_type", ivox_nearby_type_, 18);

    /* imu process parameters */
    GetROSParameter(nh, "fast_lio/max_init_count", max_init_count_, 10);

    /* share parameters */
    GetROSParameter(nh, "fast_lio/num_match_points", num_match_points_, 5);
}

