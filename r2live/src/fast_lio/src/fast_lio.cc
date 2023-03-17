#include "fast_lio.h"

FastLio::FastLio(ros::NodeHandle& nh)
{
    Init(nh);

    pub_laser_cloud_full_res_ = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100);
    pub_laser_cloud_effect_ = nh.advertise<sensor_msgs::PointCloud2>("/cloud_effected", 100);
    pub_laser_cloud_map_ = nh.advertise<sensor_msgs::PointCloud2>("/Laser_map", 100);
    pub_odom_aft_aapped_ = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 10);
    pub_path_ = nh.advertise<nav_msgs::Path>("/path", 10);

    sub_imu_ = nh.subscribe(imu_topic_, 2000000, &FastLio::ImuCbk, this, ros::TransportHints().tcpNoDelay());
    sub_pcl_ = nh.subscribe("/laser_cloud_flat", 2000000, &FastLio::FeatPointsCbk, this, ros::TransportHints().tcpNoDelay());

}

void FastLio::Init(ros::NodeHandle& nh)
{
    nh.param<std::string>("r2live/imu_topic", imu_topic_, "/livox/imu");
    GetROSParameter(nh, "fast_lio/dense_map_enable", dense_map_en_, true);
    GetROSParameter(nh, "fast_lio/lidar_time_delay", lidar_time_delay_, 0.0);
    GetROSParameter(nh, "fast_lio/max_iteration", NUM_MAX_ITERATIONS, 4);
    ros::param::get("fast_lio/map_file_path", map_file_path_);
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


    path_.header.stamp = ros::Time::now();
    path_.header.frame_id = "/world";


    FOV_DEG = fov_deg_ + 10;
    HALF_FOV_COS = std::cos((fov_deg_ + 10.0) * 0.5 * PI_M / 180.0);

    feats_undistort_.reset(new PointCloudXYZI());
    feats_down_.reset(new PointCloudXYZI());

    cube_points_add_.reset(new PointCloudXYZI());

    x_axis_point_body_ = Eigen::Vector3f(LIDAR_SP_LEN, 0.0, 0.0);
    x_axis_point_world_ = Eigen::Vector3f(LIDAR_SP_LEN, 0.0, 0.0);

    downsize_filter_map_.setLeafSize(filter_size_map_min_, filter_size_map_min_, filter_size_map_min_);
    downsize_filter_surf_.setLeafSize(filter_size_surf_min_, filter_size_surf_min_, filter_size_surf_min_);

    lio_core_ptr_ = std::make_shared<LioCore>(NUM_MAX_ITERATIONS,
                                             maximum_pt_kdtree_dis_,
                                             planar_check_dis_,
                                             long_rang_pt_dis_,
                                             maximum_res_dis_);
    point_cloud_map_ptr_ = std::make_shared<PointCloudMap>(filter_size_map_min_, cube_len_);
    lio_core_ptr_->SetPointCloudMap(point_cloud_map_ptr_);

    imu_process_ = std::make_shared<ImuProcess>();
    thread_process_ = std::thread(&FastLio::Process, this);
}

void FastLio::ImuCbk(const sensor_msgs::Imu::ConstPtr &msg_in)
{
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));
    double timestamp = msg->header.stamp.toSec();
    g_camera_lidar_queue.imu_in(timestamp);
    mtx_buffer_.lock();
    if (timestamp < last_timestamp_imu_)
    {
        ROS_ERROR("imu loop back, clear buffer");
        imu_buffer_.clear();
        flg_reset_ = true;
    }

     last_timestamp_imu_ = timestamp;
    if (g_camera_lidar_queue.m_if_acc_mul_G) 
    {
        msg->linear_acceleration.x *= G_m_s2;
        msg->linear_acceleration.y *= G_m_s2;
        msg->linear_acceleration.z *= G_m_s2;
    }
    imu_buffer_.push_back(msg);
    mtx_buffer_.unlock();
    sig_buffer_.notify_all();
}

void FastLio::FeatPointsCbk(const sensor_msgs::PointCloud2::ConstPtr &msg_in)
{
    sensor_msgs::PointCloud2::Ptr msg(new sensor_msgs::PointCloud2(*msg_in));
    msg->header.stamp = ros::Time( msg_in->header.stamp.toSec() - lidar_time_delay_);
    if (g_camera_lidar_queue.lidar_in( msg_in->header.stamp.toSec() + 0.1 ) == 0) return;
   
    mtx_buffer_.lock();
    if (msg->header.stamp.toSec() < last_timestamp_lidar_)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer_.clear();
    }

    lidar_buffer_.push_back(msg);
    last_timestamp_lidar_ = msg->header.stamp.toSec();

    mtx_buffer_.unlock();
    sig_buffer_.notify_all();
}

bool FastLio::SyncPackages(MeasureGroup &meas)
{
    if (lidar_buffer_.empty() || imu_buffer_.empty())
    {
        return false;
    }

    if (!lidar_pushed_)
    {
        meas.lidar.reset(new PointCloudXYZI());
        pcl::fromROSMsg(*(lidar_buffer_.front()), *(meas.lidar));
        meas.lidar_beg_time = lidar_buffer_.front()->header.stamp.toSec();
        lidar_end_time_ = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
        meas.lidar_end_time = lidar_end_time_;
        lidar_pushed_ = true;
    }

    if (last_timestamp_imu_ < lidar_end_time_)
    {
        return false;
    }

    double imu_time = imu_buffer_.front()->header.stamp.toSec();
    meas.imu.clear();
    while ((!imu_buffer_.empty()) && (imu_time < lidar_end_time_))
    {
        imu_time = imu_buffer_.front()->header.stamp.toSec();
        if (imu_time > lidar_end_time_ + 0.02)
            break;
        meas.imu.push_back(imu_buffer_.front());
        imu_buffer_.pop_front();
    }

    lidar_buffer_.pop_front();
    lidar_pushed_ = false;
    return true;
}

int FastLio::Process()
{
    ros::Rate rate(5000);
    g_camera_lidar_queue.m_lidar_frame_buf = &lidar_buffer_;
    while(ros::ok())
    {
        if (flg_exit_) break;
        ros::spinOnce();

        while (g_camera_lidar_queue.if_lidar_can_process() == false)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        std::unique_lock<std::mutex> lock(mutex_lio_process_);

        if (SyncPackages(Measures) == 0) continue;
        if(g_camera_lidar_queue.m_if_lidar_can_start== 0) continue;

        g_lidar_star_tim = first_lidar_time_;

        if (flg_reset_)
        {
            imu_process_->Reset();
            flg_reset_ = false;
            continue;
        }

        imu_process_->Process(Measures, g_lio_state, feats_undistort_);

        g_camera_lidar_queue.g_noise_cov_acc = imu_process_->cov_acc;
        g_camera_lidar_queue.g_noise_cov_gyro = imu_process_->cov_gyr;
        StatesGroup state_propagat(g_lio_state);

        if (feats_undistort_->empty() || (feats_undistort_ == NULL))
        {
            first_lidar_time_ = Measures.lidar_beg_time;
            g_lio_state.last_update_time = first_lidar_time_;
            std::cout << "not ready for odometry" << std::endl;
            continue;
        }

        if ((Measures.lidar_beg_time - first_lidar_time_) < INIT_TIME)
        {
            flg_EKF_inited_ = false;
            std::cout << "||||||||||Initiallizing LiDar||||||||||" << std::endl;
        }
        else
        {
            flg_EKF_inited_ = true;
        }

         /*** Segment the map in lidar FOV ***/
        LasermapFovSegment();

        downsize_filter_surf_.setInputCloud(feats_undistort_);
        downsize_filter_surf_.filter(*feats_down_);

        if (!flg_map_inited_)
        {
            point_cloud_map_ptr_->InitPointCloudMap(feats_down_);
            flg_map_inited_ = true;
            continue;
        }

        if (point_cloud_map_ptr_->GetPointsNumFromMap() < 5)
        {
            std::cout << "Insufficient map points, discard update!" << std::endl;
            continue;
        }

        lio_core_ptr_->SetEKFFlg(flg_EKF_inited_);
        lio_core_ptr_->Update(feats_down_);

        int points_size = feats_down_->points.size();
        PointCloudXYZI::Ptr feats_down_updated(new PointCloudXYZI(*feats_down_));
        for (int i = 0; i < points_size; i++)
        {
            PointTypeBodyToWorld(&(feats_down_->points[i]), &(feats_down_updated->points[i]));
        }

        point_cloud_map_ptr_->AddNewPointCloud(feats_down_updated, featsArray);
        PublishData(feats_undistort_, feats_down_);
        rate.sleep();
    }
    return 0;
}

void FastLio::LasermapFovSegment()
{
    int laserCloudValidNum = 0;

    PointBodyToWorld(x_axis_point_body_, x_axis_point_world_);

    int centerCubeI = int((g_lio_state.pos_end(0) + 0.5 * cube_len_) / cube_len_) + laserCloudCenWidth;
    int centerCubeJ = int((g_lio_state.pos_end(1) + 0.5 * cube_len_) / cube_len_) + laserCloudCenHeight;
    int centerCubeK = int((g_lio_state.pos_end(2) + 0.5 * cube_len_) / cube_len_) + laserCloudCenDepth;

    if (g_lio_state.pos_end(0) + 0.5 * cube_len_ < 0)
        centerCubeI--;
    if (g_lio_state.pos_end(1) + 0.5 * cube_len_ < 0)
        centerCubeJ--;
    if (g_lio_state.pos_end(2) + 0.5 * cube_len_ < 0)
        centerCubeK--;

    bool last_inFOV_flag = 0;
    int cube_index = 0;
    cub_needrm_.clear();
    cub_needad_.clear();
    T2[time_log_counter] = Measures.lidar_beg_time;
    double t_begin = omp_get_wtime();

    // std::cout << "centerCubeIJK: " << centerCubeI << " " << centerCubeJ << " " << centerCubeK << std::endl;

    while (centerCubeI < FOV_RANGE + 1)
    {
        for (int j = 0; j < laserCloudHeight; j++)
        {
            for (int k = 0; k < laserCloudDepth; k++)
            {
                int i = laserCloudWidth - 1;

                PointCloudXYZI::Ptr laserCloudCubeSurfPointer = featsArray[CubeInd(i, j, k)];
                last_inFOV_flag = _last_inFOV[cube_index];

                for (; i >= 1; i--)
                {
                    featsArray[CubeInd(i, j, k)] = featsArray[CubeInd(i - 1, j, k)];
                    _last_inFOV[CubeInd(i, j, k)] = _last_inFOV[CubeInd(i - 1, j, k)];
                }

                featsArray[CubeInd(i, j, k)] = laserCloudCubeSurfPointer;
                _last_inFOV[CubeInd(i, j, k)] = last_inFOV_flag;
                laserCloudCubeSurfPointer->clear();
            }
        }
        centerCubeI++;
        laserCloudCenWidth++;
    }

    while (centerCubeI >= laserCloudWidth - (FOV_RANGE + 1))
    {
        for (int j = 0; j < laserCloudHeight; j++)
        {
            for (int k = 0; k < laserCloudDepth; k++)
            {
                int i = 0;

                PointCloudXYZI::Ptr laserCloudCubeSurfPointer = featsArray[CubeInd(i, j, k)];
                last_inFOV_flag = _last_inFOV[cube_index];

                for (; i >= 1; i--)
                {
                    featsArray[CubeInd(i, j, k)] = featsArray[CubeInd(i + 1, j, k)];
                    _last_inFOV[CubeInd(i, j, k)] = _last_inFOV[CubeInd(i + 1, j, k)];
                }

                    featsArray[CubeInd(i, j, k)] = laserCloudCubeSurfPointer;
                _last_inFOV[CubeInd(i, j, k)] = last_inFOV_flag;
                laserCloudCubeSurfPointer->clear();
            }
        }

        centerCubeI--;
        laserCloudCenWidth--;
    }

    while (centerCubeJ < (FOV_RANGE + 1))
    {
        for (int i = 0; i < laserCloudWidth; i++)
        {
            for (int k = 0; k < laserCloudDepth; k++)
            {
                int j = laserCloudHeight - 1;

                PointCloudXYZI::Ptr laserCloudCubeSurfPointer = featsArray[CubeInd(i, j, k)];
                last_inFOV_flag = _last_inFOV[cube_index];

                for (; i >= 1; i--)
                {
                    featsArray[CubeInd(i, j, k)] = featsArray[CubeInd(i, j - 1, k)];
                    _last_inFOV[CubeInd(i, j, k)] = _last_inFOV[CubeInd(i, j - 1, k)];
                }

                featsArray[CubeInd(i, j, k)] = laserCloudCubeSurfPointer;
                _last_inFOV[CubeInd(i, j, k)] = last_inFOV_flag;
                laserCloudCubeSurfPointer->clear();
            }
        }

        centerCubeJ++;
        laserCloudCenHeight++;
    }

    while (centerCubeJ >= laserCloudHeight - (FOV_RANGE + 1))
    {
        for (int i = 0; i < laserCloudWidth; i++)
        {
            for (int k = 0; k < laserCloudDepth; k++)
            {
                int j = 0;
                PointCloudXYZI::Ptr laserCloudCubeSurfPointer = featsArray[CubeInd(i, j, k)];
                last_inFOV_flag = _last_inFOV[cube_index];

                for (; i >= 1; i--)
                {
                    featsArray[CubeInd(i, j, k)] = featsArray[CubeInd(i, j + 1, k)];
                    _last_inFOV[CubeInd(i, j, k)] = _last_inFOV[CubeInd(i, j + 1, k)];
                }

                featsArray[CubeInd(i, j, k)] = laserCloudCubeSurfPointer;
                _last_inFOV[CubeInd(i, j, k)] = last_inFOV_flag;
                laserCloudCubeSurfPointer->clear();
            }
        }

        centerCubeJ--;
        laserCloudCenHeight--;
    }

    while (centerCubeK < (FOV_RANGE + 1))
    {
        for (int i = 0; i < laserCloudWidth; i++)
        {
            for (int j = 0; j < laserCloudHeight; j++)
            {
                int k = laserCloudDepth - 1;
                PointCloudXYZI::Ptr laserCloudCubeSurfPointer = featsArray[CubeInd(i, j, k)];
                last_inFOV_flag = _last_inFOV[cube_index];

                for (; i >= 1; i--)
                {
                    featsArray[CubeInd(i, j, k)] = featsArray[CubeInd(i, j, k - 1)];
                    _last_inFOV[CubeInd(i, j, k)] = _last_inFOV[CubeInd(i, j, k - 1)];
                }

                featsArray[CubeInd(i, j, k)] = laserCloudCubeSurfPointer;
                _last_inFOV[CubeInd(i, j, k)] = last_inFOV_flag;
                laserCloudCubeSurfPointer->clear();
            }
        }

        centerCubeK++;
        laserCloudCenDepth++;
    }

    while (centerCubeK >= laserCloudDepth - (FOV_RANGE + 1))
    {
        for (int i = 0; i < laserCloudWidth; i++)
        {
            for (int j = 0; j < laserCloudHeight; j++)
            {
                int k = 0;
                PointCloudXYZI::Ptr laserCloudCubeSurfPointer = featsArray[CubeInd(i, j, k)];
                last_inFOV_flag = _last_inFOV[cube_index];

                for (; i >= 1; i--)
                {
                    featsArray[CubeInd(i, j, k)] = featsArray[CubeInd(i, j, k + 1)];
                    _last_inFOV[CubeInd(i, j, k)] = _last_inFOV[CubeInd(i, j, k + 1)];
                }

                featsArray[CubeInd(i, j, k)] = laserCloudCubeSurfPointer;
                _last_inFOV[CubeInd(i, j, k)] = last_inFOV_flag;
                laserCloudCubeSurfPointer->clear();
            }
        }
        centerCubeK--;
        laserCloudCenDepth--;
    }

    cube_points_add_->clear();
    memset(now_inFOV, 0, sizeof(now_inFOV));
    copy_time_ = omp_get_wtime() - t_begin;
    double fov_check_begin = omp_get_wtime();

    for (int i = centerCubeI - FOV_RANGE; i <= centerCubeI + FOV_RANGE; i++)
    {
        for (int j = centerCubeJ - FOV_RANGE; j <= centerCubeJ + FOV_RANGE; j++)
        {
            for (int k = centerCubeK - FOV_RANGE; k <= centerCubeK + FOV_RANGE; k++)
            {
                if (i >= 0 && i < laserCloudWidth &&
                    j >= 0 && j < laserCloudHeight &&
                    k >= 0 && k < laserCloudDepth)
                {
                    Eigen::Vector3f center_p(cube_len_ * (i - laserCloudCenWidth),
                                                cube_len_ * (j - laserCloudCenHeight),
                                                cube_len_ * (k - laserCloudCenDepth));

                    float check1, check2;
                    float squaredSide1, squaredSide2;
                    float ang_cos = 1;
                    bool &last_inFOV = _last_inFOV[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                    bool inFOV = CenterinFOV(center_p);

                    for (int ii = -1; (ii <= 1) && (!inFOV); ii += 2)
                    {
                        for (int jj = -1; (jj <= 1) && (!inFOV); jj += 2)
                        {
                            for (int kk = -1; (kk <= 1) && (!inFOV); kk += 2)
                            {
                                Eigen::Vector3f corner_p(cube_len_ * ii, cube_len_ * jj, cube_len_ * kk);
                                corner_p = center_p + 0.5 * corner_p;

                                inFOV = CornerinFOV(corner_p);
                            }
                        }
                    }

                    now_inFOV[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = inFOV;

                    /*** readd cubes and points ***/
                    if (inFOV)
                    {
                        int center_index = i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
                        *cube_points_add_ += *featsArray[center_index];
                        featsArray[center_index]->clear();
                        if (!last_inFOV)
                        {
                            BoxPointType cub_points;
                            for (int i = 0; i < 3; i++)
                            {
                                cub_points.vertex_max[i] = center_p[i] + 0.5 * cube_len_;
                                cub_points.vertex_min[i] = center_p[i] - 0.5 * cube_len_;
                            }
                            cub_needad_.push_back(cub_points);
                            laserCloudValidInd[laserCloudValidNum] = center_index;
                            laserCloudValidNum++;
                            // std::cout<<"readd center: "<<center_p.transpose()<<std::endl;
                        }
                    }
                }
            }
        }
    }
    /*** delete cubes ***/
    for (int i = 0; i < laserCloudWidth; i++)
    {
        for (int j = 0; j < laserCloudHeight; j++)
        {
            for (int k = 0; k < laserCloudDepth; k++)
            {
                int ind = i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
                if ((!now_inFOV[ind]) && _last_inFOV[ind])
                {
                    BoxPointType cub_points;
                    Eigen::Vector3f center_p(cube_len_ * (i - laserCloudCenWidth),
                                                cube_len_ * (j - laserCloudCenHeight),
                                                cube_len_ * (k - laserCloudCenDepth));
                    // std::cout<<"center_p: "<<center_p.transpose()<<std::endl;

                    for (int i = 0; i < 3; i++)
                    {
                        cub_points.vertex_max[i] = center_p[i] + 0.5 * cube_len_;
                        cub_points.vertex_min[i] = center_p[i] - 0.5 * cube_len_;
                    }
                    cub_needrm_.push_back(cub_points);
                }
                _last_inFOV[ind] = now_inFOV[ind];
            }
        }
    }
    fov_check_time_ = omp_get_wtime() - fov_check_begin;

    double readd_begin = omp_get_wtime();

    if (cub_needrm_.size() > 0)
        point_cloud_map_ptr_->DeletePointBoxes(cub_needrm_);
    if (cub_needad_.size() > 0)
        point_cloud_map_ptr_->AddPointBoxes(cub_needad_);
    if (cube_points_add_->points.size() > 0)
        point_cloud_map_ptr_->AddPoints(cube_points_add_);
}

int FastLio::CubeInd(const int &i, const int &j, const int &k)
{
    return (i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k);
}

bool FastLio::CenterinFOV(Eigen::Vector3f cube_p)
{
    Eigen::Vector3f dis_vec = g_lio_state.pos_end.cast<float>() - cube_p;
    float squaredSide1 = dis_vec.transpose() * dis_vec;

    if (squaredSide1 < 0.4 * cube_len_ * cube_len_)
        return true;

    dis_vec = x_axis_point_world_.cast<float>() - cube_p;
    float squaredSide2 = dis_vec.transpose() * dis_vec;

    float ang_cos = fabs(squaredSide1 <= 3) ? 1.0 : (LIDAR_SP_LEN * LIDAR_SP_LEN + squaredSide1 - squaredSide2) / (2 * LIDAR_SP_LEN * sqrt(squaredSide1));

    return ((ang_cos > HALF_FOV_COS) ? true : false);
}

bool FastLio::CornerinFOV(Eigen::Vector3f cube_p)
{
    Eigen::Vector3f dis_vec = g_lio_state.pos_end.cast<float>() - cube_p;
    float squaredSide1 = dis_vec.transpose() * dis_vec;

    dis_vec = x_axis_point_world_.cast<float>() - cube_p;
    float squaredSide2 = dis_vec.transpose() * dis_vec;

    float ang_cos = fabs(squaredSide1 <= 3) ? 1.0 : (LIDAR_SP_LEN * LIDAR_SP_LEN + squaredSide1 - squaredSide2) / (2 * LIDAR_SP_LEN * sqrt(squaredSide1));

    return ((ang_cos > HALF_FOV_COS) ? true : false);
}

void FastLio::RGBpointBodyToWorld(PointType const *const pi, pcl::PointXYZI *const po)
{
    Eigen::Vector3d p_body(pi->x, pi->y, pi->z);
    Eigen::Vector3d p_global(g_lio_state.rot_end * (p_body + Lidar_offset_to_IMU) + g_lio_state.pos_end);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;

    float intensity = pi->intensity;
    intensity = intensity - std::floor(intensity);

    int reflection_map = intensity * 10000;
}

void FastLio::PointTypeBodyToWorld(PointType const *const pi, PointType *const po)
{
    Eigen::Vector3d p_body(pi->x, pi->y, pi->z);
    Eigen::Vector3d p_global(g_lio_state.rot_end * (p_body + Lidar_offset_to_IMU) + g_lio_state.pos_end);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}


void FastLio::PublishData(PointCloudXYZI::Ptr feats_undistort, PointCloudXYZI::Ptr feats_down)
{
    // 1. 发布当前帧点云
    PublishCurrentFrame(feats_undistort, feats_down);
    // 2. 发布有效点云
    PublishEffePoints();
    // 3. 发布地图
    PublishMap();
    // 4. 发布里程计
    PublishOdom();
    // 5. tf变换
    PublishTFTransform();
    // 6. 发布路径
    PublishPath();
    // 7. 向bag包中写数据
}


void FastLio::PublishCurrentFrame(PointCloudXYZI::Ptr feats_undistort, PointCloudXYZI::Ptr feats_down)
{
    PointCloudXYZI::Ptr cur_frame_points(new PointCloudXYZI());
    cur_frame_points->clear();
    *cur_frame_points = dense_map_en_ ? (*feats_undistort) : (*feats_down);

    int points_size = cur_frame_points->points.size();

    pcl::PointXYZI temp_point;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cur_color_points(new pcl::PointCloud<pcl::PointXYZI>());
    cur_color_points->clear();

    for (int i = 0; i < points_size; i++)
    {
        RGBpointBodyToWorld(&cur_frame_points->points[i], &temp_point);
        cur_color_points->push_back(temp_point);
    }

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cur_color_points, msg);
    msg.header.stamp.fromSec(Measures.lidar_end_time);
    msg.header.frame_id = "world";       // world; camera_init
    pub_laser_cloud_full_res_.publish(msg);
}

void FastLio::PublishEffePoints()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cur_color_points(new pcl::PointCloud<pcl::PointXYZI>());
    PointCloudXYZI::Ptr laserCloudOri = lio_core_ptr_->GetLaserCloudOri();

    int points_size = laserCloudOri->points.size();

    pcl::PointXYZI temp_point;
    for (int i = 0; i < points_size; i++)
    {
        RGBpointBodyToWorld(&laserCloudOri->points[i], &temp_point);
        cur_color_points->push_back(temp_point);
    }

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cur_color_points, msg);
    msg.header.stamp.fromSec(Measures.lidar_end_time);
    msg.header.frame_id = "world";       // world; camera_init
    pub_laser_cloud_effect_.publish(msg);
}

void FastLio::PublishMap()
{
    PointCloudXYZI::Ptr feats_from_map = point_cloud_map_ptr_->GetFeatureMap();
    sensor_msgs::PointCloud2 laserCloudMap;
    pcl::toROSMsg(*feats_from_map, laserCloudMap);

    laserCloudMap.header.stamp.fromSec(Measures.lidar_end_time); //ros::Time().fromSec(last_timestamp_lidar);
    laserCloudMap.header.frame_id = "world";
    pub_laser_cloud_map_.publish(laserCloudMap);
}


void FastLio::PublishOdom()
{
    auto euler_cur = RotMtoEuler(g_lio_state.rot_end);
    nav_msgs::Odometry odomAftMapped;
    ChangeFormatData(odomAftMapped, euler_cur);
    pub_odom_aft_aapped_.publish(odomAftMapped);
}

void FastLio::PublishTFTransform()
{
    auto euler_cur = RotMtoEuler(g_lio_state.rot_end);
    nav_msgs::Odometry odomAftMapped;
    ChangeFormatData(odomAftMapped, euler_cur);
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x,
                                    odomAftMapped.pose.pose.position.y,
                                    odomAftMapped.pose.pose.position.z));
    q.setW(odomAftMapped.pose.pose.orientation.w);
    q.setX(odomAftMapped.pose.pose.orientation.x);
    q.setY(odomAftMapped.pose.pose.orientation.y);
    q.setZ(odomAftMapped.pose.pose.orientation.z);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time().fromSec(Measures.lidar_end_time), "world", "/aft_mapped"));
}


void FastLio::ChangeFormatData(nav_msgs::Odometry& odomAftMapped,const Eigen::Vector3d& euler_cur)
{
    geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(euler_cur(0), euler_cur(1), euler_cur(2));
    odomAftMapped.header.frame_id = "world";
    odomAftMapped.child_frame_id = "/aft_mapped";
    odomAftMapped.header.stamp = ros::Time::now(); //ros::Time().fromSec(last_timestamp_lidar);
    odomAftMapped.pose.pose.orientation.x = geoQuat.x;
    odomAftMapped.pose.pose.orientation.y = geoQuat.y;
    odomAftMapped.pose.pose.orientation.z = geoQuat.z;
    odomAftMapped.pose.pose.orientation.w = geoQuat.w;
    odomAftMapped.pose.pose.position.x = g_lio_state.pos_end(0);
    odomAftMapped.pose.pose.position.y = g_lio_state.pos_end(1);
    odomAftMapped.pose.pose.position.z = g_lio_state.pos_end(2);
}

void FastLio::PublishPath()
{
    auto euler_cur = RotMtoEuler(g_lio_state.rot_end);
    geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(euler_cur(0), euler_cur(1), euler_cur(2));
    geometry_msgs::PoseStamped msg_body_pose;
    msg_body_pose.header.stamp = ros::Time::now();
    msg_body_pose.header.frame_id = "/camera_odom_frame";
    msg_body_pose.pose.position.x = g_lio_state.pos_end(0);
    msg_body_pose.pose.position.y = g_lio_state.pos_end(1);
    msg_body_pose.pose.position.z = g_lio_state.pos_end(2);
    msg_body_pose.pose.orientation.x = geoQuat.x;
    msg_body_pose.pose.orientation.y = geoQuat.y;
    msg_body_pose.pose.orientation.z = geoQuat.z;
    msg_body_pose.pose.orientation.w = geoQuat.w;
    msg_body_pose.header.frame_id = "world";
    path_.poses.push_back(msg_body_pose);
    pub_path_.publish(path_);
}