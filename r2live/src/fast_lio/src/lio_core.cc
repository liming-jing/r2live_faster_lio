#include "lio_core.h"

LioCore::LioCore()
{
    Init();
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

        rematch_en_ = false;
        if (flg_EKF_converged_ || ((rematch_num_ == 0) && (iter_count_ == (num_max_iterations_ - 2))))
        {
            rematch_en_ = true;
            rematch_num_++;
        }

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

void LioCore::SetEKFFlg(bool flg_EKF_inited)
{
    flg_EKF_inited_ = flg_EKF_inited;
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
        // cout << ANSI_COLOR_RED_BOLD << "Run EKF uph" << ANSI_COLOR_RESET << endl;
        auto &&Hsub_T = Hsub.transpose();
        H_T_H.block<6, 6>(0, 0) = Hsub_T * Hsub;
        Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES> &&K_1 =
            (H_T_H + (g_lio_state.cov / LASER_POINT_COV).inverse()).inverse();
        K = K_1.block<DIM_OF_STATES, 6>(0, 0) * Hsub_T;

        auto vec = state_propagat - g_lio_state;
        solution = K * (meas_vec - Hsub * vec.block<6, 1>(0, 0));
        g_lio_state = state_propagat + solution;
        // cout << ANSI_COLOR_RED_BOLD << "Run EKF uph, vec = " << vec.head<9>().transpose() << ANSI_COLOR_RESET << endl;

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