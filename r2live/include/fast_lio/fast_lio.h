#ifndef _FAST_LIO_H_
#define _FAST_LIO_H_

#include <mutex>
#include <condition_variable>
#include <thread>
#include <csignal>
#include <ros/ros.h>
#include <Eigen/Core>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <glog/logging.h>

#include "pointcloud_map_base.h"
#include "pointcloud_ikd_map.h"
#include "pointcloud_ivox_map.h"
#include "lio_core.h"
#include "r2live/parameter_server.h"
#include "r2live/common_lib.h"
#include "imu_process.h"
#include "r2live/so3_math.h"

extern Camera_Lidar_queue g_camera_lidar_queue;
extern MeasureGroup Measures;
extern StatesGroup g_lio_state;


class FastLio {
public:
    FastLio(ros::NodeHandle& nh);
     ~FastLio(){};

private:
    int Process();
    void Init(ros::NodeHandle& nh);
    void SigHanle(int sig);
    void PointBodyToWorld(PointType const *const pi, PointType *const po);
    void LasermapFovSegment(Eigen::Vector3d pos);
    void FeatPointsCbk(const sensor_msgs::PointCloud2::ConstPtr &msg_in);
    void ImuCbk(const sensor_msgs::Imu::ConstPtr &msg_in);
    bool SyncPackages(MeasureGroup &meas);

    void PublishData(PointCloudXYZI::Ptr feats_undistort, PointCloudXYZI::Ptr feats_down);

private:
    void PublishCurrentFrame(PointCloudXYZI::Ptr feats_undistort, PointCloudXYZI::Ptr feats_down);
    void PublishEffePoints();
    void PublishMap();
    void PublishOdom();
    void PublishTFTransform();
    void PublishPath();
    
private:

    void RGBpointBodyToWorld(PointType const *const pi, pcl::PointXYZI *const po);
    void ChangeFormatData(nav_msgs::Odometry& odomAftMapped,const Eigen::Vector3d& euler_cur);
public:
    std::mutex mutex_lio_process_;
    std::shared_ptr<ImuProcess> imu_process_;
    double first_lidar_time_ = 0;

private:
    // system parameter
    bool dense_map_en_;
    double lidar_time_delay_;
    int num_max_iterations_;
    double fov_deg_;
   
    double filter_size_surf_min_;
    double filter_size_map_min_;
    double last_timestamp_imu_ = -1;
    double last_timestamp_lidar_ = -1;
    double lidar_end_time_ = 0.0;
    bool flg_reset_ = false;
    bool flg_map_inited_ = false;
    bool lidar_pushed_ = false;
    bool flg_EKF_inited_ = false;
    double cube_len_ = 0.0;
    double FOV_DEG = 0.0;

    PointCloudXYZI::Ptr feats_undistort_;
    PointCloudXYZI::Ptr feats_down_;

    nav_msgs::Path path_;

private:
    
    std::mutex mtx_buffer_;
    std::condition_variable sig_buffer_;

    std::deque<sensor_msgs::PointCloud2::ConstPtr> lidar_buffer_;
    std::deque<sensor_msgs::Imu::ConstPtr> imu_buffer_;
    
    bool flg_exit_ = false;
    std::thread thread_process_;

    std::shared_ptr<PointCloudMapBase> map_base_ptr_;
    std::shared_ptr<LioCore> lio_core_ptr_;

    std::string imu_topic_;
    ros::Publisher pub_laser_cloud_full_res_;
    ros::Publisher pub_laser_cloud_effect_;
    ros::Publisher pub_laser_cloud_map_;
    ros::Publisher pub_odom_aft_aapped_;
    ros::Publisher pub_path_;

    ros::Subscriber sub_pcl_;
    ros::Subscriber sub_imu_;

    pcl::VoxelGrid<PointType> downsize_filter_surf_;
    pcl::VoxelGrid<PointType> downsize_filter_map_;
};

#endif