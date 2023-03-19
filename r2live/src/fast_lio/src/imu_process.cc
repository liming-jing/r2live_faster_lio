#include "imu_process.h"

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
    g_lidar_star_tim_ = 0;

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