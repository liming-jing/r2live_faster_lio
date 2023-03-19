#ifndef _IMU_PROCESS_H_
#define _IMU_PROCESS_H_

#include <vector>
#include <deque>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Eigen>

#include "common_lib.h"

const double kCovStartAccDiag = 1e-10;
const double kCovStartGyroDiag = 1e-10;


class ImuProcess{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        ImuProcess();

    private:
        void Init();
        void Reset();
        void ImuInit(const MeasureGroup &meas, StatesGroup &state_inout, int &N);

    private:
        Eigen::Vector3d angvel_last_;
        Eigen::Vector3d acc_s_last_;
        Eigen::Matrix<double,DIM_OF_PROC_N,1> cov_proc_noise_;

        Eigen::Vector3d cov_acc_;
        Eigen::Vector3d cov_gyr_;

        bool b_first_frame_ = true;
        bool imu_need_init_ = true;
        double g_lidar_star_tim_ = 0;

        int init_iter_num_ = 1;
        Eigen::Vector3d mean_acc_;
        Eigen::Vector3d mean_gyr_;

        PointCloudXYZI::Ptr cur_pcl_un_;
        sensor_msgs::ImuConstPtr last_imu_;
        double start_timestamp_;
        std::deque<sensor_msgs::ImuConstPtr> v_imu_;
        std::vector<Eigen::Matrix3d> v_rot_pcl_;
        std::vector<Pose6D> imu_pose_;
};

#endif