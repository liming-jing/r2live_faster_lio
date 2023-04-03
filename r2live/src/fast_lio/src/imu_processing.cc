#include "imu_processing.h"

ImuProcess::ImuProcess()
    : b_first_frame_(true), imu_need_init_(true), start_timestamp_(-1)
{
    imu_en = true;
    init_iter_num = 1;
    cov_acc = V3D(0.1, 0.1, 0.1);
    cov_gyr = V3D(0.1, 0.1, 0.1);
    // old
    cov_acc_scale = V3D(1, 1, 1);
    cov_gyr_scale = V3D(1, 1, 1);

    cov_bias_gyr = V3D(0.1, 0.1, 0.1);
    cov_bias_acc = V3D(0.1, 0.1, 0.1);
    mean_acc = V3D(0, 0, -1.0);
    mean_gyr = V3D(0, 0, 0);
    angvel_last = Zero3d;
    Lid_offset_to_IMU = Zero3d;
    Lid_rot_to_IMU = Eye3d;
    last_imu_.reset(new sensor_msgs::Imu());
}

ImuProcess::~ImuProcess() {}

void ImuProcess::Reset()
{
    ROS_WARN("Reset ImuProcess");
    mean_acc = V3D(0, 0, -1.0);
    mean_gyr = V3D(0, 0, 0);
    angvel_last = Zero3d;
    imu_need_init_ = true;
    start_timestamp_ = -1;
    init_iter_num = 1;
    v_imu_.clear();
    IMUpose.clear();
    last_imu_.reset(new sensor_msgs::Imu());
    cur_pcl_un_.reset(new PointCloudXYZI());
}

void ImuProcess::set_extrinsic(const MD(4, 4) & T)
{
    Lid_offset_to_IMU = T.block<3, 1>(0, 3);
    Lid_rot_to_IMU = T.block<3, 3>(0, 0);
}

void ImuProcess::set_extrinsic(const V3D &transl)
{
    Lid_offset_to_IMU = transl;
    Lid_rot_to_IMU.setIdentity();
}

void ImuProcess::set_extrinsic(const V3D &transl, const M3D &rot)
{
    Lid_offset_to_IMU = transl;
    Lid_rot_to_IMU = rot;
}

void ImuProcess::set_gyr_cov_scale(const V3D &scaler)
{
    cov_gyr_scale = scaler;
}

void ImuProcess::set_acc_cov_scale(const V3D &scaler)
{
    cov_acc_scale = scaler;
}

void ImuProcess::set_gyr_bias_cov(const V3D &b_g) { cov_bias_gyr = b_g; }

void ImuProcess::set_acc_bias_cov(const V3D &b_a) { cov_bias_acc = b_a; }

void ImuProcess::IMU_init(const MeasureGroup &meas, StatesGroup &state_inout,
                          int &N)
{
    /** 1. initializing the gravity, gyro bias, acc and gyro covariance
     ** 2. normalize the acceleration measurenments to unit gravity **/
    ROS_INFO("IMU Initializing: %.1f %%", double(N) / MAX_INI_COUNT * 100);
    V3D cur_acc, cur_gyr;

    if (b_first_frame_)
    {
        Reset();
        N = 1;
        b_first_frame_ = false;
        const auto &imu_acc = meas.imu.front()->linear_acceleration;
        const auto &gyr_acc = meas.imu.front()->angular_velocity;
        mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;
        mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;
        first_lidar_time = meas.lidar_beg_time;
    }

    for (const auto &imu : meas.imu)
    {
        const auto &imu_acc = imu->linear_acceleration;
        const auto &gyr_acc = imu->angular_velocity;
        cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
        cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

        mean_acc += (cur_acc - mean_acc) / N;
        mean_gyr += (cur_gyr - mean_gyr) / N;

        cov_acc = cov_acc * (N - 1.0) / N +
                  (cur_acc - mean_acc).cwiseProduct(cur_acc - mean_acc) *
                      (N - 1.0) / (N * N);
        cov_gyr = cov_gyr * (N - 1.0) / N +
                  (cur_gyr - mean_gyr).cwiseProduct(cur_gyr - mean_gyr) *
                      (N - 1.0) / (N * N);

        // cout<<"acc norm: "<<cur_acc.norm()<<" "<<mean_acc.norm()<<endl;

        N++;
    }

    state_inout.gravity = -mean_acc / mean_acc.norm() * G_m_s2;

    state_inout.rot_end =
        Eye3d; // Exp(mean_acc.cross(V3D(0, 0, -1 / scale_gravity)));
    state_inout.bias_g = mean_gyr;

    last_imu_ = meas.imu.back();
}

void ImuProcess::NewUndistortPcl(const MeasureGroup &meas, StatesGroup &state_inout,
                                 PointCloudXYZI &pcl_out)
{
    /*** add the imu of the last frame-tail to the of current frame-head ***/
    auto v_imu = meas.imu;
    v_imu.push_front(last_imu_);
    const double &pcl_beg_time = meas.lidar_beg_time;

    SortPointCloudByOffsetTime(meas, pcl_out);

    /*** Initialize IMU pose ***/
    IMUpose.clear();
    IMUpose.push_back(set_pose6d(0.0, acc_s_last, angvel_last,
                                 state_inout.vel_end, state_inout.pos_end,
                                 state_inout.rot_end));

    /*** forward propagation at each imu point ***/
    V3D acc_imu, angvel_avr, acc_avr, vel_imu(state_inout.vel_end),
        pos_imu(state_inout.pos_end);
    M3D R_imu(state_inout.rot_end);

    double dt = 0;
    for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++)
    {
        auto &&head = *(it_imu);
        auto &&tail = *(it_imu + 1);

        if (!CalAngVelAndAccVel(angvel_avr, acc_avr, dt, head, tail, state_inout))
        {
            continue;
        }

        /* covariance propagation */
        CovariancePropagation(state_inout, angvel_avr, acc_avr, R_imu, dt);

        ImuStateUpdate(state_inout, angvel_avr, acc_avr, dt, R_imu, acc_imu, pos_imu, vel_imu);
        /* save the poses at each IMU measurements */
        angvel_last = angvel_avr;
        acc_s_last = acc_imu;
        double &&offs_t = tail->header.stamp.toSec() - pcl_beg_time;
        IMUpose.push_back(
            set_pose6d(offs_t, acc_imu, angvel_avr, vel_imu, pos_imu, R_imu));
    }

    CalImuStateFrameEnd(meas, state_inout, angvel_avr, acc_avr, R_imu, acc_imu, pos_imu, vel_imu);

    last_lidar_end_time = (pcl_out.points.back().curvature != 0) ? (pcl_beg_time + pcl_out.points.back().curvature / double(1000)) : meas.lidar_sec_time;

    UndistortLidarPoint(state_inout, pcl_out);
}

void ImuProcess::LicPointCloudUndistort(const MeasureGroup &meas, const StatesGroup &state_in, PointCloudXYZI &pcl_out)
{
    StatesGroup state_inout = state_in;
    /*** add the imu of the last frame-tail to the of current frame-head ***/
    auto v_imu = meas.imu;
    v_imu.push_front(last_imu_);
    const double &pcl_beg_time = meas.lidar_beg_time;

    SortPointCloudByOffsetTime(meas, pcl_out);

    /*** Initialize IMU pose ***/
    IMUpose.clear();
    IMUpose.push_back(set_pose6d(0.0, acc_s_last, angvel_last,
                                 state_inout.vel_end, state_inout.pos_end,
                                 state_inout.rot_end));

    /*** forward propagation at each imu point ***/
    V3D acc_imu, angvel_avr, acc_avr, vel_imu(state_inout.vel_end),
        pos_imu(state_inout.pos_end);
    M3D R_imu(state_inout.rot_end);

    double dt = 0;
    for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++)
    {
        auto &&head = *(it_imu);
        auto &&tail = *(it_imu + 1);

        if (!CalAngVelAndAccVel(angvel_avr, acc_avr, dt, head, tail, state_inout))
        {
            continue;
        }

        ImuStateUpdate(state_inout, angvel_avr, acc_avr, dt, R_imu, acc_imu, pos_imu, vel_imu);

        /* save the poses at each IMU measurements */
        angvel_last = angvel_avr;
        acc_s_last = acc_imu;
        double &&offs_t = tail->header.stamp.toSec() - pcl_beg_time;
        IMUpose.push_back(
            set_pose6d(offs_t, acc_imu, angvel_avr, vel_imu, pos_imu, R_imu));
    }

    CalImuStateFrameEnd(meas, state_inout, angvel_avr, acc_avr, R_imu, acc_imu, pos_imu, vel_imu);

    UndistortLidarPoint(state_inout, pcl_out);
}

void ImuProcess::LicStatePropagate(const MeasureGroup &meas, StatesGroup &state_inout)
{
    auto v_imu = meas.imu;
    v_imu.push_front(last_imu_);

    /*** forward propagation at each imu point ***/
    V3D acc_imu, angvel_avr, acc_avr, vel_imu(state_inout.vel_end),
        pos_imu(state_inout.pos_end);
    M3D R_imu(state_inout.rot_end);

    double dt = 0;
    for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++)
    {
        auto &&head = *(it_imu);
        auto &&tail = *(it_imu + 1);

        if (!CalAngVelAndAccVel(angvel_avr, acc_avr, dt, head, tail, state_inout))
        {
            continue;
        }

        /* covariance propagation */
        CovariancePropagation(state_inout, angvel_avr, acc_avr, R_imu, dt);

        ImuStateUpdate(state_inout, angvel_avr, acc_avr, dt, R_imu, acc_imu, pos_imu, vel_imu);
    }

    CalImuStateFrameEnd(meas, state_inout, angvel_avr, acc_avr, R_imu, acc_imu, pos_imu, vel_imu);
}

void ImuProcess::SortPointCloudByOffsetTime(const MeasureGroup &meas, PointCloudXYZI &pcl_out)
{
    /*** sort point clouds by offset time ***/
    pcl_out = *(meas.lidar);
    for (auto &it : pcl_out)
    {
        V3D P_i(it.x, it.y, it.z);
        P_i = Lid_rot_to_IMU * P_i + Lid_offset_to_IMU;
        it.x = P_i[0];
        it.y = P_i[1];
        it.z = P_i[2];
    };

    sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);
}

bool ImuProcess::CalAngVelAndAccVel(V3D &angvel_avr, V3D &acc_avr, double &dt,
                                    sensor_msgs::ImuConstPtr &head,
                                    sensor_msgs::ImuConstPtr &tail,
                                    const StatesGroup &state_inout)
{
    if (tail->header.stamp.toSec() < last_lidar_end_time)
        return false;

    angvel_avr << 0.5 * (head->angular_velocity.x + tail->angular_velocity.x),
        0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
        0.5 * (head->angular_velocity.z + tail->angular_velocity.z);
    acc_avr << 0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
        0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
        0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);

    angvel_avr -= state_inout.bias_g;
    acc_avr = acc_avr * G_m_s2 / mean_acc.norm() - state_inout.bias_a; // 原来代码

    if (head->header.stamp.toSec() < last_lidar_end_time)
    {
        dt = tail->header.stamp.toSec() - last_lidar_end_time;
    }
    else
    {
        dt = tail->header.stamp.toSec() - head->header.stamp.toSec();
    }

    return true;
}

void ImuProcess::CovariancePropagation(StatesGroup &state_inout, const V3D &angvel_avr,
                                       const V3D &acc_avr, const M3D &R_imu, double dt)
{
    MD(DIM_STATE, DIM_STATE)
    F_x, cov_w;

    M3D acc_avr_skew;
    acc_avr_skew << SKEW_SYM_MATRX(acc_avr);

    F_x.setIdentity();
    cov_w.setZero();

    F_x.block<3, 3>(0, 0) = Exp(angvel_avr, -dt);
    F_x.block<3, 3>(0, 9) = -Eye3d * dt;
    F_x.block<3, 3>(3, 6) = Eye3d * dt;
    F_x.block<3, 3>(6, 0) = -R_imu * acc_avr_skew * dt;
    F_x.block<3, 3>(6, 12) = -R_imu * dt;
    F_x.block<3, 3>(6, 15) = Eye3d * dt;

    cov_w.block<3, 3>(0, 0).diagonal() = cov_gyr * dt * dt;
    cov_w.block<3, 3>(6, 6) =
        R_imu * cov_acc.asDiagonal() * R_imu.transpose() * dt * dt;
    cov_w.block<3, 3>(9, 9).diagonal() =
        cov_bias_gyr * dt * dt; // bias gyro covariance
    cov_w.block<3, 3>(12, 12).diagonal() =
        cov_bias_acc * dt * dt; // bias acc covariance

    state_inout.cov = F_x * state_inout.cov * F_x.transpose() + cov_w;
}

void ImuProcess::ImuStateUpdate(const StatesGroup &state_inout, const V3D &angvel_avr,
                                const V3D &acc_avr, double dt, M3D &R_imu, V3D &acc_imu,
                                V3D &pos_imu, V3D &vel_imu)
{
    M3D Exp_f = Exp(angvel_avr, dt);

    /* propogation of IMU attitude */
    R_imu = R_imu * Exp_f;

    /* Specific acceleration (global frame) of IMU */
    acc_imu = R_imu * acc_avr + state_inout.gravity;

    /* propogation of IMU */
    pos_imu = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt;

    /* velocity of IMU */
    vel_imu = vel_imu + acc_imu * dt;
}

void ImuProcess::CalImuStateFrameEnd(const MeasureGroup &meas, StatesGroup &state_inout,
                                     const V3D &angvel_avr, const V3D &acc_avr, M3D &R_imu,
                                     V3D &acc_imu, V3D &pos_imu, V3D &vel_imu)
{
    auto v_imu = meas.imu;
    v_imu.push_front(last_imu_);
    const double &imu_end_time = v_imu.back()->header.stamp.toSec();
    const double &pcl_end_time = (meas.lidar->points.back().curvature != 0) ? (meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000)) : meas.lidar_sec_time;

    double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
    double dt = note * (pcl_end_time - imu_end_time);
    state_inout.vel_end = vel_imu + note * acc_imu * dt;
    state_inout.rot_end = R_imu * Exp(V3D(note * angvel_avr), dt);
    state_inout.pos_end =
        pos_imu + note * vel_imu * dt + note * 0.5 * acc_imu * dt * dt;
}

void ImuProcess::UndistortLidarPoint(const StatesGroup &state_inout, PointCloudXYZI &pcl_out)
{
    auto pos_liD_e =
        state_inout.pos_end + state_inout.rot_end * Lid_offset_to_IMU;

    M3D R_imu;
    V3D acc_imu, vel_imu, pos_imu, angvel_avr;

    /*** undistort each lidar point (backward propagation) ***/
    auto it_pcl = pcl_out.points.end() - 1;
    for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--)
    {
        auto head = it_kp - 1;
        auto tail = it_kp;
        R_imu << MAT_FROM_ARRAY(head->rot);
        acc_imu << VEC_FROM_ARRAY(head->acc);
        // cout<<"head imu acc: "<<acc_imu.transpose()<<endl;
        vel_imu << VEC_FROM_ARRAY(head->vel);
        pos_imu << VEC_FROM_ARRAY(head->pos);
        angvel_avr << VEC_FROM_ARRAY(head->gyr);

        for (; it_pcl->curvature / double(1000) > head->offset_time; it_pcl--)
        {
            double dt = it_pcl->curvature / double(1000) - head->offset_time;

            M3D R_i(R_imu * Exp(angvel_avr, dt));
            V3D T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt +
                     R_i * Lid_offset_to_IMU - pos_liD_e);

            V3D P_i(it_pcl->x, it_pcl->y, it_pcl->z);
            V3D P_compensate = state_inout.rot_end.transpose() * (R_i * P_i + T_ei);

            /// save Undistorted points and their rotation
            it_pcl->x = P_compensate(0);
            it_pcl->y = P_compensate(1);
            it_pcl->z = P_compensate(2);

            if (it_pcl == pcl_out.points.begin())
                break;
        }
    }
}

void ImuProcess::Process(const MeasureGroup &meas, StatesGroup &stat,
                         PointCloudXYZI::Ptr &cur_pcl_un_)
{
    double t1, t2, t3;

    if (meas.imu.empty() && imu_en)
    {
        return;
    }
    ROS_ASSERT(meas.lidar != nullptr);

    if (imu_need_init_ && imu_en)
    {
        IMU_init(meas, stat, init_iter_num);

        imu_need_init_ = true;

        last_imu_ = meas.imu.back();

        if (init_iter_num > MAX_INI_COUNT)
        {
            cov_acc *= pow(G_m_s2 / mean_acc.norm(), 2);
            imu_need_init_ = false;
            ROS_INFO("IMU Initials: Gravity: %.4f %.4f %.4f %.4f; state.bias_g: %.4f "
                     "%.4f %.4f; acc covarience: %.8f %.8f %.8f; gry covarience: "
                     "%.8f %.8f %.8f",
                     stat.gravity[0], stat.gravity[1], stat.gravity[2],
                     mean_acc.norm(), cov_acc_scale[0], cov_acc_scale[1],
                     cov_acc_scale[2], cov_acc[0], cov_acc[1], cov_acc[2], cov_gyr[0],
                     cov_gyr[1], cov_gyr[2]);
            cov_acc = Eye3d * cov_acc_scale;
            cov_gyr = Eye3d * cov_gyr_scale;
        }

        return;
    }

    // UndistortPcl(meas, stat, *cur_pcl_un_);
    // NewUndistortPcl(meas, stat, *cur_pcl_un_);
    LicPointCloudUndistort(meas, stat, *cur_pcl_un_);

    LicStatePropagate(meas, stat);

    last_imu_ = meas.imu.back();
    last_lidar_end_time = (meas.lidar->points.back().curvature != 0) ? (meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000)) : meas.lidar_sec_time;
}