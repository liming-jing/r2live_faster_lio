#include "voxel_mapping.h"

VoxelMapping::VoxelMapping(ros::NodeHandle &nh)
{

    p_pre = std::make_shared<Preprocess>();
    p_imu = std::make_shared<ImuProcess>();

    Init(nh);

    sub_pcl = p_pre->lidar_type == AVIA
                  ? nh.subscribe(lid_topic, 200000, &VoxelMapping::livox_pcl_cbk, this)
                  : nh.subscribe(lid_topic, 200000, &VoxelMapping::standard_pcl_cbk, this);

    sub_imu = nh.subscribe(imu_topic, 200000, &VoxelMapping::imu_cbk, this);

    pubLaserCloudFullRes =
        nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100);
    pubLaserCloudEffect =
        nh.advertise<sensor_msgs::PointCloud2>("/cloud_effected", 100);
    pubOdomAftMapped =
        nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 10);
    pubPath = nh.advertise<nav_msgs::Path>("/path", 10);

    voxel_map_pub =
        nh.advertise<visualization_msgs::MarkerArray>("/planes", 10000);
}

void VoxelMapping::Init(ros::NodeHandle &nh)
{
    // cummon params
    nh.param<string>("common/lid_topic", lid_topic, "/livox/lidar");
    nh.param<string>("common/imu_topic", imu_topic, "/livox/imu");

    // noise model params
    nh.param<double>("noise_model/ranging_cov", ranging_cov, 0.02);
    nh.param<double>("noise_model/angle_cov", angle_cov, 0.05);
    nh.param<double>("noise_model/gyr_cov_scale", gyr_cov_scale, 0.1);
    nh.param<double>("noise_model/acc_cov_scale", acc_cov_scale, 0.1);

    // imu params, current version does not support imu
    // nh.param<bool>("imu/imu_en", imu_en, false);
    nh.param<double>("imu/imu_freq", imu_freq, 200);

    nh.param<vector<double>>("imu/extrinsic_T", extrinT, vector<double>());
    nh.param<vector<double>>("imu/extrinsic_R", extrinR, vector<double>());

    // mapping algorithm params
    nh.param<int>("mapping/max_iteration", NUM_MAX_ITERATIONS, 4);
    nh.param<int>("mapping/max_points_size", max_points_size, 100);
    nh.param<int>("mapping/max_cov_points_size", max_cov_points_size, 100);
    nh.param<vector<double>>("mapping/layer_point_size", layer_point_size,
                             vector<double>());
    nh.param<int>("mapping/max_layer", max_layer, 2);
    nh.param<double>("mapping/voxel_size", max_voxel_size, 1.0);
    nh.param<double>("mapping/down_sample_size", filter_size_surf_min, 0.5);

    nh.param<double>("mapping/plannar_threshold", min_eigen_value, 0.01);

    // preprocess params
    nh.param<double>("preprocess/blind", p_pre->blind, 0.01);
    nh.param<int>("preprocess/lidar_type", p_pre->lidar_type, AVIA);

    if (p_pre->lidar_type == AVIA)
    {
        lid_topic = "/livox/lidar";
    }

    nh.param<int>("preprocess/scan_line", p_pre->N_SCANS, 16);
    nh.param<int>("preprocess/point_filter_num", p_pre->point_filter_num, 2);

    // visualization params
    nh.param<bool>("visualization/pub_voxel_map", publish_voxel_map, false);
    nh.param<int>("visualization/publish_max_voxel_layer",
                  publish_max_voxel_layer, 0);
    nh.param<bool>("visualization/pub_point_cloud", publish_point_cloud, true);
    nh.param<int>("visualization/pub_point_cloud_skip", pub_point_cloud_skip, 1);
    nh.param<bool>("visualization/dense_map_enable", dense_map_en, false);

    for (int i = 0; i < layer_point_size.size(); i++)
    {
        layer_size.push_back(layer_point_size[i]);
    }

    path.header.stamp = ros::Time::now();
    path.header.frame_id = "world";

    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min,
                                   filter_size_surf_min);

    Eigen::Vector3d extT;
    Eigen::Matrix3d extR;
    extT << extrinT[0], extrinT[1], extrinT[2];
    extR << extrinR[0], extrinR[1], extrinR[2], extrinR[3], extrinR[4],
        extrinR[5], extrinR[6], extrinR[7], extrinR[8];
    p_imu->set_extrinsic(extT, extR);

    p_imu->set_gyr_cov_scale(V3D(gyr_cov_scale, gyr_cov_scale, gyr_cov_scale));
    p_imu->set_acc_cov_scale(V3D(acc_cov_scale, acc_cov_scale, acc_cov_scale));
    p_imu->set_gyr_bias_cov(V3D(0.00001, 0.00001, 0.00001));
    p_imu->set_acc_bias_cov(V3D(0.00001, 0.00001, 0.00001));

    lidar_pushed = false;
    flg_reset = false;
    is_first_frame = true;
    flg_EKF_inited = false;
    init_map = false;

    nearest_search_en = true;
    rematch_num = 0;

    lidar_end_time = 0;
    publish_count = 0;
    last_timestamp_imu = -1.0;
    last_timestamp_lidar = -1.0;
    first_lidar_time_ = 0;
    total_residual = 0.0;
    effct_feat_num = 0;

    feats_undistort.reset(new PointCloudXYZI());
    feats_down_body.reset(new PointCloudXYZI());
    laserCloudOri.reset(new PointCloudXYZI());
    corr_normvect.reset(new PointCloudXYZI());
    laserCloudNoeffect.reset(new PointCloudXYZI());

    G.setZero();
    H_T_H.setZero();
    I_STATE.setIdentity();

    process = std::thread(&VoxelMapping::Run, this);
}

void VoxelMapping::standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    mtx_buffer.lock();
    // cout<<"got feature"<<endl;
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }
    // ROS_INFO("get point cloud at time: %.6f", msg->header.stamp.toSec());
    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(msg->header.stamp.toSec());
    last_timestamp_lidar = msg->header.stamp.toSec();

    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void VoxelMapping::livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg)
{
    mtx_buffer.lock();
    // cout << "got feature" << endl;
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }
    // ROS_INFO("get point cloud at time: %.6f", msg->header.stamp.toSec());
    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(msg->header.stamp.toSec());
    last_timestamp_lidar = msg->header.stamp.toSec();

    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void VoxelMapping::imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in)
{
    publish_count++;
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

    double timestamp = msg->header.stamp.toSec();

    mtx_buffer.lock();

    if (timestamp < last_timestamp_imu)
    {
        ROS_ERROR("imu loop back, clear buffer");
        imu_buffer.clear();
        flg_reset = true;
    }

    last_timestamp_imu = timestamp;

    imu_buffer.push_back(msg);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

bool VoxelMapping::sync_packages(MeasureGroup &meas)
{
    if (lidar_buffer.empty() || imu_buffer.empty())
    {
        return false;
    }

    /*** push a lidar scan ***/
    if (!lidar_pushed)
    {
        meas.lidar = lidar_buffer.front();
        if (meas.lidar->points.size() <= 1)
        {
            lidar_buffer.pop_front();
            return false;
        }
        if (time_buffer.size() < 2)
            return false;
        meas.lidar_beg_time = time_buffer.front();
        meas.lidar_sec_time = time_buffer[1];
        lidar_end_time = (meas.lidar->points.back().curvature) != 0 ? (meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000)) : time_buffer[1];
        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time)
    {
        return false;
    }

    /*** push imu data, and pop from imu buffer ***/
    double imu_time = imu_buffer.front()->header.stamp.toSec();
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
    {
        imu_time = imu_buffer.front()->header.stamp.toSec();
        if (imu_time > lidar_end_time + 1.0 / imu_freq)
            break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

void VoxelMapping::Run()
{
    // g_camera_lidar_queue.m_liar_frame_buf = &lidar_buffer;
    g_camera_lidar_queue.m_lidar_time_buf = &time_buffer;
    while (true)
    {

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        while (g_camera_lidar_queue.if_lidar_can_process() == false)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        std::unique_lock<std::mutex> lock(m_mutex_lio_process);

        if (!sync_packages(Measures))
            continue;

        if (g_camera_lidar_queue.m_if_lidar_can_start == 0)
            continue;

        if (flg_reset)
        {
            LOG(WARNING) << "reset when rosbag play back";
            p_imu->Reset();
            flg_reset = false;
            continue;
        }

        p_imu->Process(Measures, g_lio_state, feats_undistort);

        g_camera_lidar_queue.g_noise_cov_acc = p_imu->cov_acc;
        g_camera_lidar_queue.g_noise_cov_gyro = p_imu->cov_gyr;

        state_propagat = g_lio_state;

        if (is_first_frame)
        {
            first_lidar_time_ = Measures.lidar_beg_time;
            g_lio_state.last_update_time = first_lidar_time_;
            is_first_frame = false;
        }

        if (feats_undistort->empty() || (feats_undistort == NULL))
        {
            p_imu->first_lidar_time = first_lidar_time_;
            LOG(WARNING) << "FAST-LIO not ready";
            continue;
        }

        flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time_) < kInitTime ? false : true;

        if (flg_EKF_inited && !init_map)
        {
            InitVoxelMap();
            init_map = true;
            continue;
        }

        downSizeFilterSurf.setInputCloud(feats_undistort);
        downSizeFilterSurf.filter(*feats_down_body);

        sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list);

        std::vector<M3D> body_var;
        std::vector<M3D> crossmat_list;

        BodyVarAndCrossmatList(body_var, crossmat_list);

        IESKFUpdate(body_var, crossmat_list);

        UpdateVoxelMap(body_var, crossmat_list);

        Publish();
    }
}

void VoxelMapping::InitVoxelMap()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr world_lidar(
        new pcl::PointCloud<pcl::PointXYZI>);
    Eigen::Quaterniond q(g_lio_state.rot_end);
    transformLidar(g_lio_state, p_imu, feats_undistort, world_lidar);
    std::vector<pointWithCov> pv_list;
    for (size_t i = 0; i < world_lidar->size(); i++)
    {
        pointWithCov pv;
        pv.point << world_lidar->points[i].x, world_lidar->points[i].y,
            world_lidar->points[i].z;
        V3D point_this(feats_undistort->points[i].x,
                       feats_undistort->points[i].y,
                       feats_undistort->points[i].z);
        // if z=0, error will occur in calcBodyCov. To be solved
        if (point_this[2] == 0)
        {
            point_this[2] = 0.001;
        }
        M3D cov;
        calcBodyCov(point_this, ranging_cov, angle_cov, cov);

        point_this += Lidar_offset_to_IMU;
        M3D point_crossmat;
        point_crossmat << SKEW_SYM_MATRX(point_this);
        cov = g_lio_state.rot_end * cov * g_lio_state.rot_end.transpose() +
              (-point_crossmat) * g_lio_state.cov.block<3, 3>(0, 0) *
                  (-point_crossmat).transpose() +
              g_lio_state.cov.block<3, 3>(3, 3);
        pv.cov = cov;
        pv_list.push_back(pv);
    }

    buildVoxelMap(pv_list, max_voxel_size, max_layer, layer_size,
                  max_points_size, max_points_size, min_eigen_value,
                  voxel_map);
}

void VoxelMapping::IESKFUpdate(const std::vector<M3D> &body_var, const std::vector<M3D> &crossmat_list)
{
    rematch_num = 0;

    for (int iterCount = 0; iterCount < NUM_MAX_ITERATIONS; iterCount++)
    {
        std::vector<ptpl> ptpl_list;
        ThreeSigmaCriterion(ptpl_list, body_var, crossmat_list);
        SetLaserCloudAndCorrNormvect(ptpl_list);

        Eigen::MatrixXd Hsub(effct_feat_num, 6);
        Eigen::MatrixXd Hsub_T_R_inv(6, effct_feat_num);
        Eigen::VectorXd R_inv(effct_feat_num);
        Eigen::VectorXd meas_vec(effct_feat_num);

        CalJacobianMatrix(ptpl_list, Hsub, Hsub_T_R_inv, R_inv, meas_vec);

        Eigen::MatrixXd K(DIM_STATE, effct_feat_num);

        UpdateState(Hsub, Hsub_T_R_inv, R_inv, meas_vec, K);

        if (ConvergenceJudgements(iterCount, K, Hsub))
        {
            break;
        }
    }
}

void VoxelMapping::BodyVarAndCrossmatList(std::vector<M3D> &body_var, std::vector<M3D> &crossmat_list)
{
    for (size_t i = 0; i < feats_down_body->size(); i++)
    {
        V3D point_this(feats_down_body->points[i].x,
                       feats_down_body->points[i].y,
                       feats_down_body->points[i].z);
        if (point_this[2] == 0)
        {
            point_this[2] = 0.001;
        }
        M3D cov;
        calcBodyCov(point_this, ranging_cov, angle_cov, cov);
        M3D point_crossmat;
        point_crossmat << SKEW_SYM_MATRX(point_this);
        crossmat_list.push_back(point_crossmat);
        M3D rot_var = g_lio_state.cov.block<3, 3>(0, 0);
        M3D t_var = g_lio_state.cov.block<3, 3>(3, 3);
        body_var.push_back(cov);
    }
}

void VoxelMapping::ThreeSigmaCriterion(std::vector<ptpl> &ptpl_list, const std::vector<M3D> &body_var, const std::vector<M3D> &crossmat_list)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr world_lidar(
        new pcl::PointCloud<pcl::PointXYZI>);
    transformLidar(g_lio_state, p_imu, feats_down_body, world_lidar);

    std::vector<pointWithCov> pv_list;
    for (size_t i = 0; i < feats_down_body->size(); i++)
    {
        pointWithCov pv;
        pv.point << feats_down_body->points[i].x,
            feats_down_body->points[i].y, feats_down_body->points[i].z;
        pv.point_world << world_lidar->points[i].x, world_lidar->points[i].y,
            world_lidar->points[i].z;
        M3D cov = body_var[i];
        M3D point_crossmat = crossmat_list[i];
        M3D rot_var = g_lio_state.cov.block<3, 3>(0, 0);
        M3D t_var = g_lio_state.cov.block<3, 3>(3, 3);
        cov = g_lio_state.rot_end * cov * g_lio_state.rot_end.transpose() +
              (-point_crossmat) * rot_var * (-point_crossmat.transpose()) +
              t_var;
        pv.cov = cov;
        pv_list.push_back(pv);
    }
    auto scan_match_time_start = std::chrono::high_resolution_clock::now();
    std::vector<V3D> non_match_list;
    BuildResidualListOMP(voxel_map, max_voxel_size, 3.0, max_layer, pv_list,
                         ptpl_list, non_match_list);
}

void VoxelMapping::SetLaserCloudAndCorrNormvect(const std::vector<ptpl> &ptpl_list)
{
    laserCloudOri->clear();
    corr_normvect->clear();
    total_residual = 0.0;

    effct_feat_num = 0;
    for (int i = 0; i < ptpl_list.size(); i++)
    {
        PointType pi_body;
        PointType pi_world;
        PointType pl;
        pi_body.x = ptpl_list[i].point(0);
        pi_body.y = ptpl_list[i].point(1);
        pi_body.z = ptpl_list[i].point(2);
        pointBodyToWorld(&pi_body, &pi_world);
        pl.x = ptpl_list[i].normal(0);
        pl.y = ptpl_list[i].normal(1);
        pl.z = ptpl_list[i].normal(2);
        effct_feat_num++;
        float dis = (pi_world.x * pl.x + pi_world.y * pl.y +
                     pi_world.z * pl.z + ptpl_list[i].d);
        pl.intensity = dis;
        laserCloudOri->push_back(pi_body);
        corr_normvect->push_back(pl);
        total_residual += fabs(dis);
    }
}

void VoxelMapping::CalJacobianMatrix(const std::vector<ptpl> &ptpl_list, Eigen::MatrixXd &Hsub, Eigen::MatrixXd &Hsub_T_R_inv, Eigen::VectorXd &R_inv, Eigen::VectorXd &meas_vec)
{
    for (int i = 0; i < effct_feat_num; i++)
    {
        const PointType &laser_p = laserCloudOri->points[i];
        V3D point_this(laser_p.x, laser_p.y, laser_p.z);
        M3D cov;

        calcBodyCov(point_this, ranging_cov, angle_cov, cov);

        cov = g_lio_state.rot_end * cov * g_lio_state.rot_end.transpose();
        M3D point_crossmat;
        point_crossmat << SKEW_SYM_MATRX(point_this);
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);
        V3D point_world = g_lio_state.rot_end * point_this + g_lio_state.pos_end;
        // /*** get the normal vector of closest surface/corner ***/
        Eigen::Matrix<double, 1, 6> J_nq;
        J_nq.block<1, 3>(0, 0) = point_world - ptpl_list[i].center;
        J_nq.block<1, 3>(0, 3) = -ptpl_list[i].normal;
        double sigma_l = J_nq * ptpl_list[i].plane_cov * J_nq.transpose();
        R_inv(i) = 1.0 / (sigma_l + norm_vec.transpose() * cov * norm_vec);
        double ranging_dis = point_this.norm();
        laserCloudOri->points[i].intensity = sqrt(R_inv(i));
        laserCloudOri->points[i].normal_x =
            corr_normvect->points[i].intensity;
        laserCloudOri->points[i].normal_y = sqrt(sigma_l);
        laserCloudOri->points[i].normal_z =
            sqrt(norm_vec.transpose() * cov * norm_vec);
        laserCloudOri->points[i].curvature =
            sqrt(sigma_l + norm_vec.transpose() * cov * norm_vec);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D A(point_crossmat * g_lio_state.rot_end.transpose() * norm_vec);
        Hsub.row(i) << VEC_FROM_ARRAY(A), norm_p.x, norm_p.y, norm_p.z;
        Hsub_T_R_inv.col(i) << A[0] * R_inv(i), A[1] * R_inv(i),
            A[2] * R_inv(i), norm_p.x * R_inv(i), norm_p.y * R_inv(i),
            norm_p.z * R_inv(i);
        /*** Measuremnt: distance to the closest surface/corner ***/
        meas_vec(i) = -norm_p.intensity;
    }
}

void VoxelMapping::UpdateState(const Eigen::MatrixXd &Hsub, const Eigen::MatrixXd &Hsub_T_R_inv, const Eigen::VectorXd &R_inv, const Eigen::VectorXd &meas_vec, Eigen::MatrixXd &K)
{
    EKF_stop_flg = false;
    flg_EKF_converged = false;

    if (!flg_EKF_inited)
    {
        LOG(WARNING) << "||||||||||Initiallizing LiDar||||||||||";
        /*** only run in initialization period ***/
        MatrixXd H_init(MD(9, DIM_STATE)::Zero());
        MatrixXd z_init(VD(9)::Zero());
        H_init.block<3, 3>(0, 0) = M3D::Identity();
        H_init.block<3, 3>(3, 3) = M3D::Identity();
        H_init.block<3, 3>(6, 15) = M3D::Identity();
        z_init.block<3, 1>(0, 0) = -Log(g_lio_state.rot_end);
        z_init.block<3, 1>(0, 0) = -g_lio_state.pos_end;

        auto H_init_T = H_init.transpose();
        auto &&K_init =
            g_lio_state.cov * H_init_T *
            (H_init * g_lio_state.cov * H_init_T + 0.0001 * MD(9, 9)::Identity())
                .inverse();
        solution = K_init * z_init;

        // g_lio_state.resetpose();
        EKF_stop_flg = true;
    }
    else
    {
        auto &&Hsub_T = Hsub.transpose();
        H_T_H.block<6, 6>(0, 0) = Hsub_T_R_inv * Hsub;
        MD(DIM_STATE, DIM_STATE) &&K_1 =
            (H_T_H + (g_lio_state.cov).inverse()).inverse();
        K = K_1.block<DIM_STATE, 6>(0, 0) * Hsub_T_R_inv;
        auto vec = state_propagat - g_lio_state;
        solution = K * meas_vec + vec - K * Hsub * vec.block<6, 1>(0, 0);

        g_lio_state += solution;

        rot_add = solution.block<3, 1>(0, 0);
        t_add = solution.block<3, 1>(3, 0);

        if ((rot_add.norm() * 57.3 < 0.01) && (t_add.norm() * 100 < 0.015))
        {
            flg_EKF_converged = true;
        }
    }
}

bool VoxelMapping::ConvergenceJudgements(int iterCount, const Eigen::MatrixXd &K, const Eigen::MatrixXd &Hsub)
{
    euler_cur = RotMtoEuler(g_lio_state.rot_end);

    nearest_search_en = false;
    if (flg_EKF_converged ||
        ((rematch_num == 0) && (iterCount == (NUM_MAX_ITERATIONS - 2))))
    {
        nearest_search_en = true;
        rematch_num++;
    }

    /*** Convergence Judgements and Covariance Update ***/
    if (!EKF_stop_flg &&
        (rematch_num >= 2 || (iterCount == NUM_MAX_ITERATIONS - 1)))
    {
        if (flg_EKF_inited)
        {
            /*** Covariance Update ***/
            G.setZero();
            G.block<DIM_STATE, 6>(0, 0) = K * Hsub;
            g_lio_state.cov = (I_STATE - G) * g_lio_state.cov;

            geoQuat = tf::createQuaternionMsgFromRollPitchYaw(
                euler_cur(0), euler_cur(1), euler_cur(2));
        }
        EKF_stop_flg = true;
    }

    return EKF_stop_flg;
}

void VoxelMapping::UpdateVoxelMap(const std::vector<M3D> &body_var, const std::vector<M3D> &crossmat_list)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr world_lidar(
        new pcl::PointCloud<pcl::PointXYZI>);
    transformLidar(g_lio_state, p_imu, feats_down_body, world_lidar);
    std::vector<pointWithCov> pv_list;
    for (size_t i = 0; i < world_lidar->size(); i++)
    {
        pointWithCov pv;
        pv.point << world_lidar->points[i].x, world_lidar->points[i].y,
            world_lidar->points[i].z;
        M3D point_crossmat = crossmat_list[i];
        M3D cov = body_var[i];
        cov = g_lio_state.rot_end * cov * g_lio_state.rot_end.transpose() +
              (-point_crossmat) * g_lio_state.cov.block<3, 3>(0, 0) *
                  (-point_crossmat).transpose() +
              g_lio_state.cov.block<3, 3>(3, 3);
        pv.cov = cov;
        pv_list.push_back(pv);
    }
    std::sort(pv_list.begin(), pv_list.end(), var_contrast);
    updateVoxelMap(pv_list, max_voxel_size, max_layer, layer_size,
                   max_points_size, max_points_size, min_eigen_value,
                   voxel_map);
}

void VoxelMapping::pointBodyToWorld(PointType const *const pi, PointType *const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    p_body = p_body + Lidar_offset_to_IMU;
    V3D p_global(g_lio_state.rot_end * (p_body) + g_lio_state.pos_end);
    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void VoxelMapping::RGBpointBodyToWorld(PointType const *const pi, PointType *const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(g_lio_state.rot_end * (p_body) + g_lio_state.pos_end);
    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
    po->curvature = pi->curvature;
    po->normal_x = pi->normal_x;
    po->normal_y = pi->normal_y;
    po->normal_z = pi->normal_z;
    float intensity = pi->intensity;
    intensity = intensity - floor(intensity);

    int reflection_map = intensity * 10000;
}

/**
 * 发布消息
 */
void VoxelMapping::Publish()
{
    publish_odometry(pubOdomAftMapped);
    publish_path(pubPath);

    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(
        tf::Vector3(g_lio_state.pos_end(0), g_lio_state.pos_end(1), g_lio_state.pos_end(2)));
    q.setW(geoQuat.w);
    q.setX(geoQuat.x);
    q.setY(geoQuat.y);
    q.setZ(geoQuat.z);
    transform.setRotation(q);

    pcl::PointCloud<pcl::PointXYZI>::Ptr world_lidar(
        new pcl::PointCloud<pcl::PointXYZI>);
    transformLidar(g_lio_state, p_imu, feats_down_body, world_lidar);
    sensor_msgs::PointCloud2 pub_cloud;
    pcl::toROSMsg(*world_lidar, pub_cloud);
    pub_cloud.header.stamp =
        ros::Time::now(); //.fromSec(last_timestamp_lidar);
    pub_cloud.header.frame_id = "world";

    if (publish_point_cloud)
    {
        publish_frame_world(pubLaserCloudFullRes, pub_point_cloud_skip);
    }

    if (publish_voxel_map)
    {
        pubVoxelMap(voxel_map, publish_max_voxel_layer, voxel_map_pub);
    }

    publish_effect(pubLaserCloudEffect);
}

void VoxelMapping::publish_frame_world(const ros::Publisher &pubLaserCloudFullRes,
                                       const int point_skip)
{
    PointCloudXYZI::Ptr laserCloudFullRes(dense_map_en ? feats_undistort
                                                       : feats_down_body);
    int size = laserCloudFullRes->points.size();
    PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));
    for (int i = 0; i < size; i++)
    {
        RGBpointBodyToWorld(&laserCloudFullRes->points[i],
                            &laserCloudWorld->points[i]);
    }
    PointCloudXYZI::Ptr laserCloudWorldPub(new PointCloudXYZI);
    for (int i = 0; i < size; i += point_skip)
    {
        laserCloudWorldPub->points.push_back(laserCloudWorld->points[i]);
    }
    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudWorldPub, laserCloudmsg);
    laserCloudmsg.header.stamp =
        ros::Time::now(); //.fromSec(last_timestamp_lidar);
    laserCloudmsg.header.frame_id = "world";
    pubLaserCloudFullRes.publish(laserCloudmsg);
}

void VoxelMapping::publish_effect_world(const ros::Publisher &pubLaserCloudEffect,
                                        const ros::Publisher &pubPointWithCov,
                                        const std::vector<ptpl> &ptpl_list)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr effect_cloud_world(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(effct_feat_num, 1));
    visualization_msgs::MarkerArray ma_line;
    visualization_msgs::Marker m_line;
    m_line.type = visualization_msgs::Marker::LINE_LIST;
    m_line.action = visualization_msgs::Marker::ADD;
    m_line.ns = "lines";
    m_line.color.a = 0.5; // Don't forget to set the alpha!
    m_line.color.r = 1.0;
    m_line.color.g = 1.0;
    m_line.color.b = 1.0;
    m_line.scale.x = 0.01;
    m_line.pose.orientation.w = 1.0;
    m_line.header.frame_id = "world";
    for (int i = 0; i < ptpl_list.size(); i++)
    {
        Eigen::Vector3d p_c = ptpl_list[i].point;
        Eigen::Vector3d p_w = g_lio_state.rot_end * (p_c) + g_lio_state.pos_end;
        pcl::PointXYZRGB pi;
        pi.x = p_w[0];
        pi.y = p_w[1];
        pi.z = p_w[2];

        effect_cloud_world->points.push_back(pi);
        m_line.points.clear();
        geometry_msgs::Point p;
        p.x = p_w[0];
        p.y = p_w[1];
        p.z = p_w[2];
        m_line.points.push_back(p);
        p.x = ptpl_list[i].center(0);
        p.y = ptpl_list[i].center(1);
        p.z = ptpl_list[i].center(2);
        m_line.points.push_back(p);
        ma_line.markers.push_back(m_line);
        m_line.id++;
    }
    int max_num = 20000;
    for (int i = ptpl_list.size(); i < max_num; i++)
    {
        m_line.color.a = 0;
        ma_line.markers.push_back(m_line);
        m_line.id++;
    }
    pubPointWithCov.publish(ma_line);

    sensor_msgs::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*effect_cloud_world, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp =
        ros::Time::now(); //.fromSec(last_timestamp_lidar);
    laserCloudFullRes3.header.frame_id = "world";
    pubLaserCloudEffect.publish(laserCloudFullRes3);
}

void VoxelMapping::publish_no_effect(const ros::Publisher &pubLaserCloudNoEffect)
{
    sensor_msgs::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudNoeffect, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp =
        ros::Time::now(); //.fromSec(last_timestamp_lidar);
    laserCloudFullRes3.header.frame_id = "world";
    pubLaserCloudNoEffect.publish(laserCloudFullRes3);
}

void VoxelMapping::publish_effect(const ros::Publisher &pubLaserCloudEffect)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr effect_cloud_world(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(effct_feat_num, 1));
    for (int i = 0; i < effct_feat_num; i++)
    {
        RGBpointBodyToWorld(&laserCloudOri->points[i], &laserCloudWorld->points[i]);
        pcl::PointXYZRGB pi;
        pi.x = laserCloudWorld->points[i].x;
        pi.y = laserCloudWorld->points[i].y;
        pi.z = laserCloudWorld->points[i].z;
        float v = laserCloudWorld->points[i].intensity / 100;
        v = 1.0 - v;
        uint8_t r, g, b;
        mapJet(v, 0, 1, r, g, b);
        pi.r = r;
        pi.g = g;
        pi.b = b;
        effect_cloud_world->points.push_back(pi);
    }

    sensor_msgs::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp =
        ros::Time::now(); //.fromSec(last_timestamp_lidar);
    laserCloudFullRes3.header.frame_id = "world";
    pubLaserCloudEffect.publish(laserCloudFullRes3);
}

void VoxelMapping::publish_odometry(const ros::Publisher &pubOdomAftMapped)
{
    odomAftMapped.header.frame_id = "world";
    odomAftMapped.child_frame_id = "aft_mapped";
    odomAftMapped.header.stamp =
        ros::Time::now(); // ros::Time().fromSec(last_timestamp_lidar);
    set_posestamp(odomAftMapped.pose.pose);
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(
        tf::Vector3(g_lio_state.pos_end(0), g_lio_state.pos_end(1), g_lio_state.pos_end(2)));
    q.setW(geoQuat.w);
    q.setX(geoQuat.x);
    q.setY(geoQuat.y);
    q.setZ(geoQuat.z);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp,
                                          "world", "aft_mapped"));

    pubOdomAftMapped.publish(odomAftMapped);
}

void VoxelMapping::publish_mavros(const ros::Publisher &mavros_pose_publisher)
{
    msg_body_pose.header.stamp = ros::Time::now();
    msg_body_pose.header.frame_id = "camera_odom_frame";
    set_posestamp(msg_body_pose.pose);
    mavros_pose_publisher.publish(msg_body_pose);
}

void VoxelMapping::publish_path(const ros::Publisher pubPath)
{
    set_posestamp(msg_body_pose.pose);
    msg_body_pose.header.stamp = ros::Time::now();
    msg_body_pose.header.frame_id = "world";
    path.poses.push_back(msg_body_pose);
    pubPath.publish(path);
}