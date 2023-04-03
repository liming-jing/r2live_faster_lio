#ifndef _IMU_PROCESSING_H_
#define _IMU_PROCESSING_H_

#include <Eigen/Eigen>
#include <cmath>
#include <common_lib.h>
#include <condition_variable>
#include <csignal>
#include <deque>
#include <eigen_conversions/eigen_msg.h>
#include <fstream>
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <mutex>
#include <nav_msgs/Odometry.h>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <so3_math.h>
#include <tf/transform_broadcaster.h>
#include <thread>
#include <r2live/States.h>

/// *************Preconfiguration

#define MAX_INI_COUNT (10)

const inline bool time_list(PointType &x, PointType &y)
{
    return (x.curvature < y.curvature);
};

/// *************IMU Process and undistortion
class ImuProcess
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ImuProcess();
    ~ImuProcess();

    void Reset();
    void Reset(double start_timestamp, const sensor_msgs::ImuConstPtr &lastimu);
    void set_extrinsic(const V3D &transl, const M3D &rot);
    void set_extrinsic(const V3D &transl);
    void set_extrinsic(const MD(4, 4) & T);
    void set_gyr_cov_scale(const V3D &scaler);
    void set_acc_cov_scale(const V3D &scaler);
    void set_gyr_bias_cov(const V3D &b_g);
    void set_acc_bias_cov(const V3D &b_a);
    void Process(const MeasureGroup &meas, StatesGroup &state,
                 PointCloudXYZI::Ptr &pcl_un_);

    ros::NodeHandle nh;
    ofstream fout_imu;
    V3D cov_acc;
    V3D cov_gyr;
    V3D cov_acc_scale;
    V3D cov_gyr_scale;
    V3D cov_bias_gyr;
    V3D cov_bias_acc;
    M3D Lid_rot_to_IMU;
    V3D Lid_offset_to_IMU;
    double first_lidar_time;
    bool imu_en;

    double last_lidar_end_time = 0;

private:
    void IMU_init(const MeasureGroup &meas, StatesGroup &state, int &N);

    void NewUndistortPcl(const MeasureGroup &meas, StatesGroup &state_inout,
                         PointCloudXYZI &pcl_in_out);

private:
    void LicPointCloudUndistort(const MeasureGroup &meas, const StatesGroup &state_in, PointCloudXYZI &pcl_out);

    void LicStatePropagate(const MeasureGroup &meas, StatesGroup &state_inout);

private:
    /**
     * 计算 angvel_avr 和 acc_avr 以及 dt
     */
    bool CalAngVelAndAccVel(V3D &angvel_avr, V3D &acc_avr, double &dt,
                            sensor_msgs::ImuConstPtr &head,
                            sensor_msgs::ImuConstPtr &tail,
                            const StatesGroup &state_inout);

    /**
     * 计算系统的协方差
     */
    void CovariancePropagation(StatesGroup &state_inout, const V3D &angvel_avr,
                               const V3D &acc_avr, const M3D &R_imu, double dt);

    /**
     * 更新IMU 旋转 加速度 位置 和速度信息
     */
    void ImuStateUpdate(const StatesGroup &state_inout, const V3D &angvel_avr,
                        const V3D &acc_avr, double dt, M3D &R_imu, V3D &acc_imu,
                        V3D &pos_imu, V3D &vel_imu);

    /**
     * 计算IMU最后时刻的的 旋转 加速度 位置 和速度信息
     */
    void CalImuStateFrameEnd(const MeasureGroup &meas, StatesGroup &state_inout,
                             const V3D &angvel_avr, const V3D &acc_avr, M3D &R_imu,
                             V3D &acc_imu, V3D &pos_imu, V3D &vel_imu);

    void UndistortLidarPoint(const StatesGroup &state_inout, PointCloudXYZI &pcl_out);

    void SortPointCloudByOffsetTime(const MeasureGroup &meas, PointCloudXYZI &pcl_out);

    PointCloudXYZI::Ptr cur_pcl_un_;
    sensor_msgs::ImuConstPtr last_imu_;
    deque<sensor_msgs::ImuConstPtr> v_imu_;
    vector<Pose6D> IMUpose;
    vector<M3D> v_rot_pcl_;

    V3D mean_acc;
    V3D mean_gyr;
    V3D angvel_last;
    V3D acc_s_last;
    double start_timestamp_;
    double time_last_scan_;
    int init_iter_num = 1;
    bool b_first_frame_ = true;
    bool imu_need_init_ = true;
};

#endif