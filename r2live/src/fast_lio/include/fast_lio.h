#ifndef _FAST_LIO_H_
#define _FAST_LIO_H_

#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <so3_math.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <opencv/cv.h>
#include <common_lib.h>
#include "IMU_Processing.hpp"
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
#include "pointcloud_map.h"
#include "lio_core.h"

#define MAXN 360000

const int laserCloudWidth = 48;
const int laserCloudHeight = 48;
const int laserCloudDepth = 48;
const int laserCloudNum = laserCloudWidth * laserCloudHeight * laserCloudDepth;

#define INIT_TIME (0)
#define LASER_POINT_COV (0.00015)    
#define NUM_MATCH_POINTS (5)

extern Camera_Lidar_queue g_camera_lidar_queue;
extern MeasureGroup Measures;
extern StatesGroup g_lio_state;
extern double g_lidar_star_tim;

class FastLio {
public:
    FastLio(ros::NodeHandle& nh);
     ~FastLio(){};

private:
    int Process();
    void Init(ros::NodeHandle& nh);
    void SigHanle(int sig);
    void PointBodyToWorld(PointType const *const pi, PointType *const po);
    void RGBpointBodyToWorld(PointType const *const pi, pcl::PointXYZI *const po);
    int CubeInd(const int &i, const int &j, const int &k);
    bool CenterinFOV(Eigen::Vector3f cube_p);
    bool CornerinFOV(Eigen::Vector3f cube_p);
    void LasermapFovSegment();
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

    int CubeInd(const int &i, const int &j, const int &k);
    void RGBpointBodyToWorld(PointType const *const pi, pcl::PointXYZI *const po);
    void ChangeFormatData(nav_msgs::Odometry& odomAftMapped,const Eigen::Vector3d& euler_cur);
private:
    // system parameter
    bool dense_map_en_;
    double lidar_time_delay_;
    int NUM_MAX_ITERATIONS;
    std::string map_file_path_;
    double fov_deg_;
    double filter_size_corner_min_;
    double filter_size_surf_min_;
    double filter_size_surf_min_z_;
    double filter_size_map_min_;
    double first_lidar_time_ = 0;
    double cube_len_;
    double maximum_pt_kdtree_dis_;
    double maximum_res_dis_;
    double planar_check_dis_;
    double long_rang_pt_dis_;
    bool if_publish_feature_map_;
    double last_timestamp_imu_ = -1;
    double last_timestamp_lidar_ = -1;
    double lidar_end_time_ = 0.0;
    bool flg_reset_ = false;
    bool flg_map_inited_ = false;
    bool lidar_pushed_ = false;
    bool flg_EKF_inited_ = false;
    double cube_len_ = 0.0;
    double T1[MAXN], T2[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN];
    int time_log_counter = 0;
    int FOV_RANGE = 4;
    double copy_time_, readd_time_, fov_check_time_, readd_box_time_, delete_box_time_;

    int laserCloudCenWidth = 24;
    int laserCloudCenHeight = 24;
    int laserCloudCenDepth = 24;

    PointCloudXYZI::Ptr featsArray[laserCloudNum];
    bool _last_inFOV[laserCloudNum];
    bool now_inFOV[laserCloudNum];
    bool cube_updated[laserCloudNum];
    int laserCloudValidInd[laserCloudNum];
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudFullResColor; //(new pcl::PointCloud<pcl::PointXYZI>());

    std::vector<BoxPointType> cub_needrm_;
    std::vector<BoxPointType> cub_needad_;

    PointCloudXYZI::Ptr feats_undistort_;
    PointCloudXYZI::Ptr feats_down_;

    PointCloudXYZI::Ptr cube_points_add_;

    Eigen::Vector3f x_axis_point_body_; //(LIDAR_SP_LEN, 0.0, 0.0);
    Eigen::Vector3f x_axis_point_world_; //(LIDAR_SP_LEN, 0.0, 0.0);

    nav_msgs::Path path_;

private:
    std::mutex mutex_lio_process_;
    std::mutex mtx_buffer_;
    std::condition_variable sig_buffer_;
    std::shared_ptr<ImuProcess> imu_process_;
    std::string imu_topic_;

    std::deque<sensor_msgs::PointCloud2::ConstPtr> lidar_buffer_;
    std::deque<sensor_msgs::Imu::ConstPtr> imu_buffer_;
    
    bool flg_exit_ = false;
    std::thread thread_process_;

    std::shared_ptr<PointCloudMap> point_cloud_map_ptr_;
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