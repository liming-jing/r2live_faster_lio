#ifndef _IMU_PROCESS_H_
#define _IMU_PROCESS_H_

#include <vector>
#include <deque>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Eigen>

#include "common_lib.h"

const int kMaxInitCount = 10;

const double kCovStartAccDiag = 1e-10;
const double kCovStartGyroDiag = 1e-10;
const double kCovOmegaNoiseDiag = 1e-2;
const double kCovAccNoiseDiag = 1e-5;
const double kCovGyroNoiseDiag = 1e-5;
const double kCovBiasAccNoiseDiag = 0.2;
const double kCovBiasGyroNoiseDiag = 0.2;

bool check_state(StatesGroup &state_inout);

class ImuProcess{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        ImuProcess();

        void Process(const MeasureGroup &meas, StatesGroup &stat, PointCloudXYZI::Ptr cur_pcl_un_);
        void Reset();

    private:
        void Init();
        void ImuInit(const MeasureGroup &meas, StatesGroup &state_inout, int &N);
        void LicStatePropagate(const MeasureGroup &meas, StatesGroup &state_inout);
        void LicPointCloudUndistort(const MeasureGroup &meas, const StatesGroup &state_inout, PointCloudXYZI &pcl_out);
    
    public:
        StatesGroup ImuPreintegration(const StatesGroup & state_in, 
                                      std::deque<sensor_msgs::Imu::ConstPtr> & v_imu, 
                                      int if_multiply_g = 1, double end_pose_dt = 0);

    private:
        static inline bool time_list(PointType &x, PointType &y) {return (x.curvature < y.curvature);}

    public:
        Eigen::Vector3d cov_acc_;
        Eigen::Vector3d cov_gyr_;

    private:
        Eigen::Vector3d angvel_last_;
        Eigen::Vector3d acc_s_last_;
        Eigen::Matrix<double,DIM_OF_PROC_N,1> cov_proc_noise_;

        bool b_first_frame_ = true;
        bool imu_need_init_ = true;

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