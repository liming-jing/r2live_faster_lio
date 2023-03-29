#include "fast_lio/lio_core.h"

LioCore::LioCore()
{
    Init();
    LOG(INFO) << "LioCore 初始化完成...";
}

void LioCore::Init()
{

    ParameterServer* para_server = ParameterServer::GetInstance();
    num_max_iterations_ = para_server->GetNumMaxIterations();
    maximum_pt_kdtree_dis_ = para_server->GetMaximumPtKdtreeDis();
    planar_check_dis_ = para_server->GetPlanarCheckDis();
    long_rang_pt_dis_ = para_server->GetLongRangPtDis();
    maximum_res_dis_ = para_server->GetMaximumResDis();
    num_match_points_ = para_server->GetNumMatchPoints();

    laser_cloud_ori_.reset(new PointCloudXYZI());
    coeff_sel_.reset(new PointCloudXYZI());

    ranging_cov_ = para_server->GetRangingCov();
    angle_cov_ = para_server->GetAngleCov();
    max_voxel_size_ = para_server->GetMaxVoxelSize();
    max_layer_ = para_server->GetMaxLayer();
    max_points_size_ = para_server->GetMaxPointsSize();
    min_eigen_value_ = para_server->GetMinEigenValue();

    G.setZero();
    H_T_H.setZero();
    I_STATE.setIdentity();
}

void LioCore::SetMap(std::shared_ptr<PointCloudMapBase> map_base_ptr)
{
    map_base_ptr_ = map_base_ptr;
}

void LioCore::ReSetData(PointCloudXYZI::Ptr current_frame)
{
    int points_size = current_frame->points.size();
    point_selected_surf_.resize(points_size, true);
    point_search_ind_surf_.resize(points_size);
    nearest_points_.resize(points_size);

    coeff_sel_tmpt_.reset(new PointCloudXYZI(*current_frame));
    feats_down_updated_.reset(new PointCloudXYZI(*current_frame));
    res_last_.resize(points_size, 1000.0);

    flg_EKF_inited_ = ParameterServer::GetInstance()->GetFlagEKFInited();
}

void LioCore::PointBodyToWorld(PointType const *const pi, PointType *const po)
{
    Eigen::Vector3d p_body(pi->x, pi->y, pi->z);
    Eigen::Vector3d p_global(g_lio_state.rot_end * (p_body + Lidar_offset_to_IMU) + g_lio_state.pos_end);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void LioCore::PCASolver(PointVector& points_near, double ori_pt_dis, 
                        double& maximum_pt_range, const PointType &pointSel_tmpt,
                        int index)
{
    cv::Mat matA0(num_match_points_, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matB0(num_match_points_, 1, CV_32F, cv::Scalar::all(-1));
    cv::Mat matX0(num_match_points_, 1, CV_32F, cv::Scalar::all(0));

    for (int j = 0; j < num_match_points_; j++)
    {
        matA0.at<float>(j, 0) = points_near[j].x;
        matA0.at<float>(j, 1) = points_near[j].y;
        matA0.at<float>(j, 2) = points_near[j].z;
    }

    cv::solve(matA0, matB0, matX0, cv::DECOMP_QR);

    float pa = matX0.at<float>(0, 0);
    float pb = matX0.at<float>(1, 0);
    float pc = matX0.at<float>(2, 0);
    float pd = 1;

    //ps is the norm of the plane norm_vec vector
    //pd is the distance from point to plane
    float ps = sqrt(pa * pa + pb * pb + pc * pc);
    pa /= ps;
    pb /= ps;
    pc /= ps;
    pd /= ps;

    bool planeValid = true;
    for (int j = 0; j < num_match_points_; j++)
    {           
        // ANCHOR -  Planar check
        if (fabs(pa * points_near[j].x +
                    pb * points_near[j].y +
                    pc * points_near[j].z + pd) > planar_check_dis_) // Raw 0.0
        {
            // ANCHOR - Far distance pt processing
            if (ori_pt_dis < maximum_pt_range * 0.90 || (ori_pt_dis < long_rang_pt_dis_))
            {
                planeValid = false;
                point_selected_surf_[index] = false;
                break;
            }
        }
    }

    if (planeValid)
    {
        //loss fuction
        float pd2 = pa * pointSel_tmpt.x + pb * pointSel_tmpt.y + pc * pointSel_tmpt.z + pd;
        //if(fabs(pd2) > 0.1) continue;
        float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointSel_tmpt.x * pointSel_tmpt.x + pointSel_tmpt.y * pointSel_tmpt.y + pointSel_tmpt.z * pointSel_tmpt.z));
        // ANCHOR -  Point to plane distance
        //if ((s > 0.80)) // && ((std::abs(pd2) - res_last[i]) < 3 * res_mean_last)) // Ori: 0.90
        double acc_distance = (ori_pt_dis < long_rang_pt_dis_) ? maximum_res_dis_: 1.0;
        //double acc_distance = m_maximum_res_dis;
        if(pd2 < acc_distance)
        {
            point_selected_surf_[index] = true;
            coeff_sel_tmpt_->points[index].x = pa;
            coeff_sel_tmpt_->points[index].y = pb;
            coeff_sel_tmpt_->points[index].z = pc;
            coeff_sel_tmpt_->points[index].intensity = pd2;
            res_last_[index] = std::abs(pd2);                               
        }
        else
        {
            point_selected_surf_[index] = false;
        }
    }
}


void LioCore::Update(PointCloudXYZI::Ptr current_frame)
{
    switch (ParameterServer::GetInstance()->GetMapMethod())
    {
    case kIkdTreeMap:
    case kIvoxMap:
        UpdateNoVoxelMap(current_frame);
        break;
    case kVoxelMap:
        UpdateVoxelMap(current_frame);
        break;
    
    default:
        UpdateNoVoxelMap(current_frame);
        break;
    }
}

 void LioCore::UpdateNoVoxelMap(PointCloudXYZI::Ptr current_frame)
 {
    ReSetData(current_frame);
    int points_size = current_frame->points.size();
    double maximum_pt_range = 0.0;
    rematch_en_ = false;
    rematch_num_ = 0;
    for (iter_count_ = 0; iter_count_ < num_max_iterations_; iter_count_++)
    {
        laser_cloud_ori_->clear();
        coeff_sel_->clear();

        for (int i = 0; i < points_size; i++)
        {
            PointType &pointOri_tmpt = current_frame->points[i];
            double ori_pt_dis = sqrt(pointOri_tmpt.x * pointOri_tmpt.x + pointOri_tmpt.y * pointOri_tmpt.y + pointOri_tmpt.z * pointOri_tmpt.z);
            maximum_pt_range = std::max(ori_pt_dis, maximum_pt_range);
            PointType &pointSel_tmpt = feats_down_updated_->points[i];
           
            PointBodyToWorld(&pointOri_tmpt, &pointSel_tmpt);
            std::vector<float> pointSearchSqDis_surf;
            auto &points_near = nearest_points_[i];

            if (iter_count_ == 0 || rematch_en_)
            {
                bool point_selected_surf = true;
                map_base_ptr_->NearestSearch(pointSel_tmpt, points_near, num_match_points_, point_selected_surf);
                point_selected_surf_[i] = point_selected_surf;
            }

            if (point_selected_surf_[i] == false) continue;

            PCASolver(points_near, ori_pt_dis, maximum_pt_range, pointSel_tmpt, i);
        }

        double total_residual = 0.0;
        laser_cloud_sel_num_ = 0;
        for (int i = 0; i < coeff_sel_tmpt_->points.size(); i++)
        {
            if (point_selected_surf_[i] && (res_last_[i] <= 2.0))
            {
                laser_cloud_ori_->push_back(current_frame->points[i]);
                coeff_sel_->push_back(coeff_sel_tmpt_->points[i]);
                total_residual += res_last_[i];
                laser_cloud_sel_num_++;
            }
        }
        res_mean_last_ = total_residual / laser_cloud_sel_num_;

        Eigen::MatrixXd Hsub(laser_cloud_sel_num_, 6);
        Eigen::VectorXd meas_vec(laser_cloud_sel_num_);
        Hsub.setZero();
        CalculateJacobianMatrix(current_frame, Hsub, meas_vec);
        IEKFUpdateState(Hsub, meas_vec);

        if (IEKFUpdateCovariance(Hsub))
        {
            break;
        }
    }
 }

void LioCore::CalculateJacobianMatrix(PointCloudXYZI::Ptr current_frame, Eigen::MatrixXd& Hsub, Eigen::VectorXd& meas_vec)
{
    for (int i = 0; i < laser_cloud_sel_num_; i++)
    {
        const PointType &laser_p = laser_cloud_ori_->points[i];
        Eigen::Vector3d point_this(laser_p.x, laser_p.y, laser_p.z);
        point_this += Lidar_offset_to_IMU;
        Eigen::Matrix3d point_crossmat;
        point_crossmat << SKEW_SYM_MATRX(point_this);

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = coeff_sel_->points[i];
        Eigen::Vector3d norm_vec(norm_p.x, norm_p.y, norm_p.z);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        Eigen::Vector3d A(point_crossmat * g_lio_state.rot_end.transpose() * norm_vec);
        Hsub.row(i) << VEC_FROM_ARRAY(A), norm_p.x, norm_p.y, norm_p.z;

        /*** Measuremnt: distance to the closest surface/corner ***/
        meas_vec(i) = -norm_p.intensity;
    }
}

void LioCore::IEKFUpdateState(Eigen::MatrixXd& Hsub, Eigen::VectorXd& meas_vec)
{
    Eigen::Vector3d rot_add, t_add, v_add, bg_add, ba_add, g_add;
    Eigen::Matrix<double, DIM_OF_STATES, 1> solution;
    StatesGroup state_propagat(g_lio_state);
    K.resize(DIM_OF_STATES, laser_cloud_sel_num_);
    
    if (!flg_EKF_inited_)
    {
        std::cout << ANSI_COLOR_RED_BOLD << "Run EKF init" << ANSI_COLOR_RESET << std::endl;
        /*** only run in initialization period ***/
        Eigen::MatrixXd H_init(Eigen::Matrix<double, 9, DIM_OF_STATES>::Zero());
        Eigen::MatrixXd z_init(Eigen::Matrix<double, 9, 1>::Zero());
        H_init.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        H_init.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();
        H_init.block<3, 3>(6, 15) = Eigen::Matrix3d::Identity();
        z_init.block<3, 1>(0, 0) = -Log(g_lio_state.rot_end);
        z_init.block<3, 1>(0, 0) = -g_lio_state.pos_end;

        auto H_init_T = H_init.transpose();
        auto &&K_init = g_lio_state.cov * H_init_T * (H_init * g_lio_state.cov * H_init_T + 0.0001 * Eigen::Matrix<double, 9, 9>::Identity()).inverse();
        solution = K_init * z_init;

        solution.block<9, 1>(0, 0).setZero();
        g_lio_state += solution;
        g_lio_state.cov = (Eigen::MatrixXd::Identity(DIM_OF_STATES, DIM_OF_STATES) - K_init * H_init) * g_lio_state.cov;
    }
    else {
        auto &&Hsub_T = Hsub.transpose();
        H_T_H.block<6, 6>(0, 0) = Hsub_T * Hsub;
        Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES> &&K_1 =
            (H_T_H + (g_lio_state.cov / 0.00015).inverse()).inverse(); 
        K = K_1.block<DIM_OF_STATES, 6>(0, 0) * Hsub_T;

        auto vec = state_propagat - g_lio_state;
        solution = K * (meas_vec - Hsub * vec.block<6, 1>(0, 0));
        g_lio_state = state_propagat + solution;

        rot_add = solution.block<3, 1>(0, 0);
        t_add = solution.block<3, 1>(3, 0);

        flg_EKF_converged_ = false;

        if (((rot_add.norm() * 57.3 - delta_R_) < 0.01) && ((t_add.norm() * 100 - delta_T_) < 0.015))
        {
            flg_EKF_converged_ = true;
        }

        delta_R_ = rot_add.norm() * 57.3;
        delta_T_ = t_add.norm() * 100;
    }
}

bool LioCore::IEKFUpdateCovariance(Eigen::MatrixXd& Hsub)
{

    rematch_en_ = false;
    if (flg_EKF_converged_ || ((rematch_num_ == 0) && (iter_count_ == (num_max_iterations_ - 2))))
    {
        rematch_en_ = true;
        rematch_num_++;
    }

    if (rematch_num_ >= 2 || (iter_count_ == num_max_iterations_ - 1)) // Fast lio ori version.
    {
        if (flg_EKF_inited_)
        {
            /*** Covariance Update ***/
            G.block<DIM_OF_STATES, 6>(0, 0) = K * Hsub;
            g_lio_state.cov = (I_STATE - G) * g_lio_state.cov;
        }
        return true;
    }
    return false;
}


void LioCore::UpdateVoxelMap(PointCloudXYZI::Ptr current_frame)
{
    // 1. 计算每个点的协方差

    std::vector<Eigen::Matrix3d> body_var;
    std::vector<Eigen::Matrix3d> crossmat_list;

    printf_func
    CalcBodyCovAndCrossmat(current_frame, body_var, crossmat_list);
    printf_func

    map_base_ptr_->SetParameters(body_var);

    printf_func

    for (iter_count_ = 0; iter_count_ < num_max_iterations_; iter_count_++)
    {
        // 2. 3 sigma criterion
        std::vector<pointWithCov> pv_list;
        printf_func
        OrganizeData(current_frame, body_var, crossmat_list, pv_list);

        printf_func 
        // 3. 构建残差
        std::vector<ptpl> ptpl_list;
        std::vector<Eigen::Vector3d> non_match_list;
        map_base_ptr_->GetResidualList(pv_list, ptpl_list, non_match_list);
        printf_func

        // 4. 求解雅克比矩阵
        Eigen::MatrixXd Hsub, Hsub_T_R_inv;
        Eigen::VectorXd R_inv, meas_vec;
        CalculateJacobianMatrix(ptpl_list, Hsub, Hsub_T_R_inv, R_inv, meas_vec);
        printf_func
        // 5. 系统状态更新
        IEKFUpdateStateForVoxelMap(ptpl_list, Hsub, Hsub_T_R_inv, meas_vec);
        printf_func
        // 6. 收敛性判断
        if (IEKFUpdateCovariance(Hsub))
        {
            break;
        }
        printf_func
    }
    printf_func
}

void LioCore::CalcBodyCovAndCrossmat(PointCloudXYZI::Ptr current_frame, 
                                    std::vector<Eigen::Matrix3d>& body_var, 
                                    std::vector<Eigen::Matrix3d>& crossmat_list)
{

    for (size_t i = 0; i < current_frame->size(); i++)
    {
        Eigen::Vector3d point_this(current_frame->points[i].x, current_frame->points[i].y, current_frame->points[i].z);
        if (point_this[2] == 0) 
        {
            point_this[2] = 0.001;
        }
        Eigen::Matrix3d cov;
        
        CalcBodyCov(point_this, ranging_cov_, angle_cov_, cov);

        Eigen::Matrix3d point_crossmat;
        point_crossmat << SKEW_SYM_MATRX(point_this);
        crossmat_list.push_back(point_crossmat);

        body_var.push_back(cov);
    }
}

void LioCore::OrganizeData(PointCloudXYZI::Ptr current_frame, 
                          const std::vector<Eigen::Matrix3d>& body_var, 
                          const std::vector<Eigen::Matrix3d>& crossmat_list,
                          std::vector<pointWithCov>& pv_list)
{
    for (size_t i = 0, id = current_frame->points.size(); i != id; i++)
    {
        pcl::PointXYZINormal p_c = current_frame->points[i];
        Eigen::Vector3d p(p_c.x, p_c.y, p_c.z);
        p = g_lio_state.rot_end * p + g_lio_state.pos_end;

        pcl::PointXYZI pi_world;
        pi_world.x = p(0);
        pi_world.y = p(1);
        pi_world.z = p(2);
        pi_world.intensity = p_c.intensity;

        pointWithCov pv;
        pv.point << current_frame->points[i].x, current_frame->points[i].y, current_frame->points[i].z;
        pv.point_world << pi_world.x, pi_world.y, pi_world.z;

        Eigen::Matrix3d point_crossmat = crossmat_list[i];
        Eigen::Matrix3d rot_var = g_lio_state.cov.block<3, 3>(0, 0);
        Eigen::Matrix3d t_var = g_lio_state.cov.block<3, 3>(3, 3);

        Eigen::Matrix3d cov = body_var[i];
        cov = g_lio_state.rot_end * cov * g_lio_state.rot_end.transpose() +
            (-point_crossmat) * rot_var * (-point_crossmat.transpose()) +
            t_var;
        pv.cov = cov;

        pv_list.push_back(pv);
    }
}

void LioCore::CalculateJacobianMatrix(const std::vector<ptpl>& ptpl_list,
                                     Eigen::MatrixXd& Hsub,
                                     Eigen::MatrixXd& Hsub_T_R_inv,
                                     Eigen::VectorXd& R_inv,
                                     Eigen::VectorXd& meas_vec)
{
    size_t effct_feat_num = ptpl_list.size();

    Hsub.resize(effct_feat_num, 6);
    Hsub_T_R_inv.resize(6, effct_feat_num);
    R_inv.resize(effct_feat_num);
    meas_vec.resize(effct_feat_num);

    laser_cloud_ori_->clear();

    for (size_t i = 0; i != effct_feat_num; i++)
    {
        PointType pi_body;
        pi_body.x = ptpl_list[i].point(0);
        pi_body.y = ptpl_list[i].point(1);
        pi_body.z = ptpl_list[i].point(2);

        laser_cloud_ori_->push_back(pi_body);

        PointType pi_world;
        PointTypeBodyToWorld(&pi_body, &pi_world);
        
        Eigen::Vector3d point_this(pi_body.x, pi_body.y, pi_body.z);
        Eigen::Matrix3d cov;
        CalcBodyCov(point_this, ranging_cov_, angle_cov_, cov);

        cov = g_lio_state.rot_end * cov * g_lio_state.rot_end.transpose();

        Eigen::Matrix3d point_crossmat;
        point_crossmat << SKEW_SYM_MATRX(point_this);

        PointType norm_p;
        norm_p.x = ptpl_list[i].normal(0);
        norm_p.y = ptpl_list[i].normal(1);
        norm_p.z = ptpl_list[i].normal(2);
        norm_p.intensity = pi_world.x * norm_p.x + pi_world.y * norm_p.y + pi_world.z * norm_p.z + ptpl_list[i].d;


        Eigen::Vector3d norm_vec(norm_p.x, norm_p.y, norm_p.z);
        Eigen::Vector3d point_world = g_lio_state.rot_end * point_this + g_lio_state.pos_end;

        // /*** get the normal vector of closest surface/corner ***/
        Eigen::Matrix<double, 1, 6> J_nq;
        J_nq.block<1, 3>(0, 0) = point_world - ptpl_list[i].center;
        J_nq.block<1, 3>(0, 3) = -ptpl_list[i].normal;
        double sigma_l = J_nq * ptpl_list[i].plane_cov * J_nq.transpose();
        R_inv(i) = 1.0 / (sigma_l + norm_vec.transpose() * cov * norm_vec);
        double ranging_dis = point_this.norm();
        laser_cloud_ori_->points[i].intensity = sqrt(R_inv(i));
        laser_cloud_ori_->points[i].normal_x = norm_p.intensity;
        laser_cloud_ori_->points[i].normal_y = sqrt(sigma_l);
        laser_cloud_ori_->points[i].normal_z = sqrt(norm_vec.transpose() * cov * norm_vec);
        laser_cloud_ori_->points[i].curvature = sqrt(sigma_l + norm_vec.transpose() * cov * norm_vec);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        Eigen::Vector3d A(point_crossmat * g_lio_state.rot_end.transpose() * norm_vec);
        Hsub.row(i) << VEC_FROM_ARRAY(A), norm_p.x, norm_p.y, norm_p.z;
        Hsub_T_R_inv.col(i) << A[0] * R_inv(i), A[1] * R_inv(i),
            A[2] * R_inv(i), norm_p.x * R_inv(i), norm_p.y * R_inv(i),
            norm_p.z * R_inv(i);
        /*** Measuremnt: distance to the closest surface/corner ***/
        meas_vec(i) = -norm_p.intensity;
    }
}

void LioCore::IEKFUpdateStateForVoxelMap(const std::vector<ptpl>& ptpl_list,
                                         const Eigen::MatrixXd& Hsub,
                                         const Eigen::MatrixXd& Hsub_T_R_inv,
                                         const Eigen::VectorXd& meas_vec)
{
    K.resize(DIM_OF_STATES, ptpl_list.size());
    Eigen::Matrix<double, DIM_OF_STATES, 1> solution;
    StatesGroup state_propagat(g_lio_state);
    flg_EKF_inited_ = ParameterServer::GetInstance()->GetFlagEKFInited();

    if (!flg_EKF_inited_) 
    {

        /*** only run in initialization period ***/
        Eigen::MatrixXd H_init(Eigen::Matrix<double, 9, DIM_OF_STATES>::Zero());
        Eigen::MatrixXd z_init(Eigen::Matrix<double, 9, 1>::Zero());
        H_init.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        H_init.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();
        H_init.block<3, 3>(6, 15) = Eigen::Matrix3d::Identity();
        z_init.block<3, 1>(0, 0) = -Log(g_lio_state.rot_end);
        z_init.block<3, 1>(0, 0) = -g_lio_state.pos_end;

        auto H_init_T = H_init.transpose();
        auto &&K_init = g_lio_state.cov * H_init_T * (H_init * g_lio_state.cov * H_init_T + 0.0001 * Eigen::Matrix<double, 9, 9>::Identity()).inverse();
        solution = K_init * z_init;

        solution.block<9, 1>(0, 0).setZero();
        g_lio_state += solution;
        g_lio_state.cov = (Eigen::MatrixXd::Identity(DIM_OF_STATES, DIM_OF_STATES) - K_init * H_init) * g_lio_state.cov;
        
    } 
    else 
    {
        auto &&Hsub_T = Hsub.transpose();
        H_T_H.block<6, 6>(0, 0) = Hsub_T_R_inv * Hsub;
        Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES> &&K_1 =
            (H_T_H + (g_lio_state.cov / 0.00015).inverse()).inverse(); 
        K = K_1.block<DIM_OF_STATES, 6>(0, 0) * Hsub_T;

        auto vec = state_propagat - g_lio_state;
        solution = K * meas_vec + vec - K * Hsub * vec.block<6, 1>(0, 0);

        g_lio_state += solution;

        Eigen::Vector3d rot_add = solution.block<3, 1>(0, 0);
        Eigen::Vector3d t_add = solution.block<3, 1>(3, 0);

        if ((rot_add.norm() * 57.3 < 0.01) && (t_add.norm() * 100 < 0.015)) 
        {
            flg_EKF_converged_ = true;
        }
    }
}


void LioCore::TransformPointBody2World(const PointCloudXYZI::Ptr &point_cloud_body, 
                                      pcl::PointCloud<pcl::PointXYZI>::Ptr &point_cloud_world)
{
    point_cloud_world->clear();

    for (size_t i = 0; i < point_cloud_body->size(); i++) {

        pcl::PointXYZINormal p_c = point_cloud_body->points[i];
        Eigen::Vector3d p(p_c.x, p_c.y, p_c.z);
        p = g_lio_state.rot_end * p + g_lio_state.pos_end;

        pcl::PointXYZI pi;
        pi.x = p(0);
        pi.y = p(1);
        pi.z = p(2);
        pi.intensity = p_c.intensity;

        point_cloud_world->points.push_back(pi);
    }
}

void LioCore::PointTypeBodyToWorld(PointType const *const pi, PointType *const po)
{
    Eigen::Vector3d p_body(pi->x, pi->y, pi->z);
    Eigen::Vector3d p_global(g_lio_state.rot_end * (p_body + Lidar_offset_to_IMU) + g_lio_state.pos_end);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}