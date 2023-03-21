#ifndef _FAST_LIO_H_
#define _FAST_LIO_H_

#include <mutex>
#include <condition_variable>
#include <math.h>
#include <thread>
#include <csignal>
#include <unistd.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <opencv/cv.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <opencv2/core/eigen.hpp>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <glog/logging.h>

// #include "pointcloud_map.h"
#include "pointcloud_ivox_map.h"
#include "lio_core.h"
#include "parameter_server.h"
#include "common_lib.h"
#include "imu_process.h"
#include "so3_math.h"

const int laserCloudWidth = 48;
const int laserCloudHeight = 48;
const int laserCloudDepth = 48;
const int laserCloudNum = laserCloudWidth * laserCloudHeight * laserCloudDepth;
const float kMovThreshold = 1.5f;
const float kDetRange = 300.0f;
const double kInitTime = 0.0f;

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
    template <typename T>
    void PointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi, Eigen::Matrix<T, 3, 1> &po)
    {
        Eigen::Vector3d p_body(pi[0], pi[1], pi[2]);
        Eigen::Vector3d p_global(g_lio_state.rot_end * (p_body + Lidar_offset_to_IMU) + g_lio_state.pos_end);
        po[0] = p_global(0);
        po[1] = p_global(1);
        po[2] = p_global(2);
    }

    void RGBpointBodyToWorld(PointType const *const pi, pcl::PointXYZI *const po);
    void PointTypeBodyToWorld(PointType const *const pi, PointType *const po);
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
   
    PointCloudXYZI::Ptr featsArray_[laserCloudNum];

    std::vector<BoxPointType> cub_needrm_;

    BoxPointType local_map_points_;
    bool local_map_init_;

    PointCloudXYZI::Ptr feats_undistort_;
    PointCloudXYZI::Ptr feats_down_;

    Eigen::Vector3f x_axis_point_body_; //(LIDAR_SP_LEN, 0.0, 0.0);
    Eigen::Vector3f x_axis_point_world_; //(LIDAR_SP_LEN, 0.0, 0.0);

    nav_msgs::Path path_;

private:
    
    std::mutex mtx_buffer_;
    std::condition_variable sig_buffer_;

    std::deque<sensor_msgs::PointCloud2::ConstPtr> lidar_buffer_;
    std::deque<sensor_msgs::Imu::ConstPtr> imu_buffer_;
    
    bool flg_exit_ = false;
    std::thread thread_process_;

    // std::shared_ptr<PointCloudMap> point_cloud_map_ptr_;
    std::shared_ptr<PointCloudIvoxMap> point_cloud_ivox_ptr_;
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