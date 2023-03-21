#include "fast_lio/imu_process.h"

ImuProcess::ImuProcess()
{
    Init();
}

void ImuProcess::Init()
{
    b_first_frame_ = true;
    imu_need_init_ = true;
    last_imu_ = nullptr;
    start_timestamp_ = -1;
    init_iter_num_ = 1;
    
    cov_acc_ = Eigen::Vector3d(kCovStartAccDiag, kCovStartAccDiag, kCovStartAccDiag);
    cov_gyr_ = Eigen::Vector3d(kCovStartGyroDiag, kCovStartGyroDiag, kCovStartGyroDiag);
    mean_acc_ = Eigen::Vector3d(0, 0, -9.805);
    mean_gyr_ = Eigen::Vector3d(0, 0, 0);
    angvel_last_ = Zero3d;
    cov_proc_noise_ = Eigen::Matrix<double, DIM_OF_PROC_N, 1>::Zero();

    cur_pcl_un_.reset(new PointCloudXYZI());
}

void ImuProcess::Reset()
{
    angvel_last_ = Zero3d;
    cov_proc_noise_ = Eigen::Matrix<double, DIM_OF_PROC_N, 1>::Zero();

    cov_acc_ = Eigen::Vector3d(kCovStartAccDiag, kCovStartAccDiag, kCovStartAccDiag);
    cov_gyr_ = Eigen::Vector3d(kCovStartGyroDiag, kCovStartGyroDiag, kCovStartGyroDiag);
    mean_acc_ = Eigen::Vector3d(0, 0, -9.805);
    mean_gyr_ = Eigen::Vector3d(0, 0, 0);

    imu_need_init_ = true;
    b_first_frame_ = true;
    init_iter_num_ = 1;
    last_imu_ = nullptr;
    start_timestamp_ = -1;
    v_imu_.clear();
    imu_pose_.clear();

    cur_pcl_un_.reset(new PointCloudXYZI());
}

void ImuProcess::Process(const MeasureGroup &meas, StatesGroup &stat, PointCloudXYZI::Ptr cur_pcl_un_)
{
    if (meas.imu.empty())
    {
        std::cout << "no imu data" << std::endl;
        return;
    }

    ROS_ASSERT(meas.lidar != nullptr);

    if (imu_need_init_)
    {
        ImuInit(meas, stat, init_iter_num_);
        last_imu_ = meas.imu.back();

        int max_init_count = ParameterServer::GetInstance()->GetMaxInitCount(); 
        if (init_iter_num_ > max_init_count)
        {
            imu_need_init_ = false;
        }

        return;
    }

    LicPointCloudUndistort(meas, stat, *cur_pcl_un_);
    LicStatePropagate(meas, stat);

    last_imu_ = meas.imu.back();
}

void ImuProcess::ImuInit(const MeasureGroup &meas, StatesGroup &state_inout, int &N)
{
    Eigen::Vector3d cur_acc, cur_gyr;
    if (b_first_frame_)
    {
        Reset();
        N = 1;
        b_first_frame_ = false;
    }

    for (const auto &imu : meas.imu)
    {
        const auto &imu_acc = imu->linear_acceleration;
        const auto &gyr_acc = imu->angular_velocity;
        cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
        cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

        mean_acc_ += (cur_acc - mean_acc_) / N;
        mean_gyr_ += (cur_gyr - mean_gyr_) / N;

        cov_acc_ = cov_acc_ * (N - 1.0) / N + (cur_acc - mean_acc_).cwiseProduct(cur_acc - mean_acc_) * (N - 1.0) / (N * N);
        cov_gyr_ = cov_gyr_ * (N - 1.0) / N + (cur_gyr - mean_gyr_).cwiseProduct(cur_gyr - mean_gyr_) * (N - 1.0) / (N * N);
        N++;
    }

    cov_acc_ = Eigen::Vector3d(kCovStartAccDiag, kCovStartAccDiag, kCovStartAccDiag);
    cov_gyr_ = Eigen::Vector3d(kCovStartGyroDiag, kCovStartGyroDiag, kCovStartGyroDiag);
    state_inout.gravity = Eigen::Vector3d(0, 0, 9.805);
    state_inout.rot_end = Eye3d;
    state_inout.bias_g = mean_gyr_;    
}

void ImuProcess::LicStatePropagate(const MeasureGroup &meas, StatesGroup &state_inout)
{
    auto v_imu = meas.imu;
    v_imu.push_front(last_imu_);
    const double &imu_end_time = v_imu.back()->header.stamp.toSec();
    const double &pcl_beg_time = meas.lidar_beg_time;

    PointCloudXYZI pcl_out = *(meas.lidar);
    std::sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);
    const double &pcl_end_time = pcl_beg_time + pcl_out.points.back().curvature * 1e-3;
    double end_pose_dt = pcl_end_time - imu_end_time;

    state_inout = ImuPreintegration(state_inout, v_imu, 1, end_pose_dt);
    last_imu_ = meas.imu.back();
}

void ImuProcess::LicPointCloudUndistort(const MeasureGroup &meas, const StatesGroup &state_in, PointCloudXYZI &pcl_out)
{
    StatesGroup state_inout = state_in;
    auto v_imu = meas.imu;
    v_imu.push_front(last_imu_);

    const double &imu_end_time = v_imu.back()->header.stamp.toSec();
    const double &pcl_beg_time = meas.lidar_beg_time;
    /*** sort point clouds by offset time ***/
    pcl_out = *(meas.lidar);
    std::sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);
    const double &pcl_end_time = pcl_beg_time + pcl_out.points.back().curvature * 1e-3;
  
    /*** Initialize IMU pose ***/
    imu_pose_.clear();
    imu_pose_.push_back(set_pose6d(0.0, acc_s_last_, angvel_last_, state_inout.vel_end, state_inout.pos_end, state_inout.rot_end));

    /*** forward propagation at each imu point ***/
    Eigen::Vector3d acc_imu, angvel_avr, acc_avr, vel_imu(state_inout.vel_end), pos_imu(state_inout.pos_end);
    Eigen::Matrix3d R_imu(state_inout.rot_end);
    Eigen::MatrixXd F_x(Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES>::Identity());
    Eigen::MatrixXd cov_w(Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES>::Zero());
    double dt = 0;
  
    for (auto it_imu = v_imu.begin(); it_imu != (v_imu.end() - 1); it_imu++)
    {
        auto &&head = *(it_imu);
        auto &&tail = *(it_imu + 1);

        angvel_avr << 0.5 * (head->angular_velocity.x + tail->angular_velocity.x),
            0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
            0.5 * (head->angular_velocity.z + tail->angular_velocity.z);
        acc_avr << 0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
            0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
            0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);

        angvel_avr -= state_inout.bias_g;
        acc_avr = acc_avr  - state_inout.bias_a;

        dt = tail->header.stamp.toSec() - head->header.stamp.toSec();
        /* covariance propagation */

        Eigen::Matrix3d acc_avr_skew;
        Eigen::Matrix3d Exp_f = Exp(angvel_avr, dt);
        acc_avr_skew << SKEW_SYM_MATRX(acc_avr);

        /* propagation of IMU attitude */
        R_imu = R_imu * Exp_f;

        /* Specific acceleration (global frame) of IMU */
        acc_imu = R_imu * acc_avr - state_inout.gravity;

        /* propagation of IMU */
        pos_imu = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt;

        /* velocity of IMU */
        vel_imu = vel_imu + acc_imu * dt;

        /* save the poses at each IMU measurements */
        angvel_last_ = angvel_avr;
        acc_s_last_ = acc_imu;
        double &&offs_t = tail->header.stamp.toSec() - pcl_beg_time;
        imu_pose_.push_back(set_pose6d(offs_t, acc_imu, angvel_avr, vel_imu, pos_imu, R_imu));
    }

    /*** calculated the pos and attitude prediction at the frame-end ***/
    dt = pcl_end_time - imu_end_time;
    state_inout.vel_end = vel_imu + acc_imu * dt;
    state_inout.rot_end = R_imu * Exp(angvel_avr, dt);
    state_inout.pos_end = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt;

    Eigen::Vector3d pos_liD_e = state_inout.pos_end + state_inout.rot_end * Lidar_offset_to_IMU;
  

    /*** undistort each lidar point (backward propagation) ***/
    auto it_pcl = pcl_out.points.end() - 1;
    for (auto it_kp = imu_pose_.end() - 1; it_kp != imu_pose_.begin(); it_kp--)
    {
        auto head = it_kp - 1;
        auto tail = it_kp;
        R_imu << MAT_FROM_ARRAY(head->rot);
        acc_imu << VEC_FROM_ARRAY(head->acc);
        vel_imu << VEC_FROM_ARRAY(head->vel);
        pos_imu << VEC_FROM_ARRAY(head->pos);
        angvel_avr << VEC_FROM_ARRAY(head->gyr);

        for (; it_pcl->curvature * 1e-3 > head->offset_time; it_pcl--)
        {
            dt = it_pcl->curvature * 1e-3 - head->offset_time;

            Eigen::Matrix3d R_i(R_imu * Exp(angvel_avr, dt));
            Eigen::Vector3d T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt + R_i * Lidar_offset_to_IMU - pos_liD_e);

            Eigen::Vector3d P_i(it_pcl->x, it_pcl->y, it_pcl->z);
            Eigen::Vector3d P_compensate = state_inout.rot_end.transpose() * (R_i * P_i + T_ei);

            /// save Undistorted points and their rotation
            it_pcl->x = P_compensate(0);
            it_pcl->y = P_compensate(1);
            it_pcl->z = P_compensate(2);

            if (it_pcl == pcl_out.points.begin())
                break;
        }
    }
}

StatesGroup ImuProcess::ImuPreintegration(const StatesGroup & state_in, 
                                          std::deque<sensor_msgs::Imu::ConstPtr> & v_imu, 
                                          int if_multiply_g, double end_pose_dt)
{
    StatesGroup state_inout = state_in;
    if (check_state(state_inout))
    {
        state_inout.display(state_inout, "state_inout");
        state_in.display(state_in, "state_in");
    }
    Eigen::Vector3d acc_imu(0, 0, 0), angvel_avr(0, 0, 0), acc_avr(0, 0, 0), vel_imu(0, 0, 0), pos_imu(0, 0, 0);
    vel_imu = state_inout.vel_end;
    pos_imu = state_inout.pos_end;
    Eigen::Matrix3d R_imu(state_inout.rot_end);
    Eigen::MatrixXd F_x(Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES>::Identity());
    Eigen::MatrixXd cov_w(Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES>::Zero());
    double dt = 0;
    int if_first_imu = 1;
   
    for (std::deque<sensor_msgs::Imu::ConstPtr>::iterator it_imu = v_imu.begin(); it_imu != (v_imu.end() - 1); it_imu++)
    {
        sensor_msgs::Imu::ConstPtr head = *(it_imu);
        sensor_msgs::Imu::ConstPtr tail = *(it_imu+1);

        angvel_avr << 0.5 * (head->angular_velocity.x + tail->angular_velocity.x),
            0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
            0.5 * (head->angular_velocity.z + tail->angular_velocity.z);
        acc_avr << 0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
            0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
            0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);

        angvel_avr -= state_inout.bias_g;
        
        acc_avr = acc_avr  - state_inout.bias_a;
        

        if (tail->header.stamp.toSec() < state_inout.last_update_time)
        {
            continue;
        }

        if (if_first_imu)
        {
            if_first_imu = 0;
            dt = tail->header.stamp.toSec() - state_inout.last_update_time;
        }
        else
        {
            dt = tail->header.stamp.toSec() - head->header.stamp.toSec();
        }
        if (dt > 0.05)
        {
            scope_color(ANSI_COLOR_RED_BOLD);
            for (int i = 0; i < 1; i++)
            {
                cout << __FILE__ << ", " << __LINE__ << ", dt = " << dt << endl;
            }
            dt = 0.05;
        }

        /* covariance propagation */
        Eigen::Matrix3d acc_avr_skew;
        Eigen::Matrix3d Exp_f = Exp(angvel_avr, dt);
        acc_avr_skew << SKEW_SYM_MATRX(acc_avr);
        Eigen::Matrix3d Jr_omega_dt = Eigen::Matrix3d::Identity();
        F_x.block<3, 3>(0, 0) = Exp_f.transpose();
        F_x.block<3, 3>(0, 9) = -Jr_omega_dt * dt;
        F_x.block<3, 3>(3, 3) = Eye3d; // Already the identity.
        F_x.block<3, 3>(3, 6) = Eye3d * dt;
        F_x.block<3, 3>(6, 0) = -R_imu * acc_avr_skew * dt;
        F_x.block<3, 3>(6, 12) = -R_imu * dt;
        F_x.block<3, 3>(6, 15) = Eye3d * dt;

        Eigen::Matrix3d cov_acc_diag, cov_gyr_diag, cov_omega_diag;
        cov_omega_diag = Eigen::Vector3d(kCovOmegaNoiseDiag, kCovOmegaNoiseDiag, kCovOmegaNoiseDiag).asDiagonal();
        cov_acc_diag = Eigen::Vector3d(kCovAccNoiseDiag, kCovAccNoiseDiag, kCovAccNoiseDiag).asDiagonal();
        cov_gyr_diag = Eigen::Vector3d(kCovGyroNoiseDiag, kCovGyroNoiseDiag, kCovGyroNoiseDiag).asDiagonal();
        // cov_w.block<3, 3>(0, 0) = cov_omega_diag * dt * dt;
        cov_w.block<3, 3>(0, 0) = Jr_omega_dt * cov_omega_diag * Jr_omega_dt * dt * dt;
        cov_w.block<3, 3>(3, 3) = R_imu * cov_gyr_diag * R_imu.transpose() * dt * dt;
        cov_w.block<3, 3>(6, 6) = cov_acc_diag * dt * dt;
        cov_w.block<3, 3>(9, 9).diagonal() = Eigen::Vector3d(kCovBiasGyroNoiseDiag, kCovBiasGyroNoiseDiag, kCovBiasGyroNoiseDiag) * dt * dt; // bias gyro covariance
        cov_w.block<3, 3>(12, 12).diagonal() = Eigen::Vector3d(kCovBiasAccNoiseDiag, kCovBiasAccNoiseDiag, kCovBiasAccNoiseDiag) * dt * dt;  // bias acc covariance

        state_inout.cov = F_x * state_inout.cov * F_x.transpose() + cov_w;

        R_imu = R_imu * Exp_f;
        acc_imu = R_imu * acc_avr - state_inout.gravity;
        pos_imu = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt;
        vel_imu = vel_imu + acc_imu * dt;
        angvel_last_ = angvel_avr;
        acc_s_last_ = acc_imu;
    }
    dt = end_pose_dt;

    state_inout.last_update_time = v_imu.back()->header.stamp.toSec() + dt;
    if (dt > 0.1)
    {
        scope_color(ANSI_COLOR_RED_BOLD);
        for (int i = 0; i < 1; i++)
        {
            cout << __FILE__ << ", " << __LINE__ << "dt = " << dt << endl;
        }
        dt = 0.1;
    }
    state_inout.vel_end = vel_imu + acc_imu * dt;
    state_inout.rot_end = R_imu * Exp(angvel_avr, dt);
    state_inout.pos_end = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt;

    return state_inout;
}

bool check_state(StatesGroup &state_inout)
{
    bool is_fail = false;
    for (int idx = 0; idx < 3; idx++)
    {
        if (fabs(state_inout.vel_end(idx)) > 10)
        {
            is_fail = true;
            scope_color(ANSI_COLOR_RED_BG);
            for (int i = 0; i < 10; i++)
            {
                cout << __FILE__ << ", " << __LINE__ << ", check_state fail !!!! " << state_inout.vel_end.transpose() << endl;
            }
            state_inout.vel_end(idx) = 0.0;
        }
    }
    return is_fail;
}