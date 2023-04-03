#ifndef _VOXEL_MAPPING_H_
#define _VOXEL_MAPPING_H_

#include "imu_processing.h"
#include "preprocess.h"
#include "voxel_map_util.h"
#include "common_lib.h"
#include "so3_math.h"

#include <Eigen/Core>
#include <csignal>
#include <fstream>
#include <math.h>
#include <mutex>
#include <thread>
#include <unistd.h>
#include <deque>
#include <glog/logging.h>

#include <r2live/States.h>
#include <livox_ros_driver/CustomMsg.h>

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf2_msgs/TFMessage.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

constexpr double kInitTime = 0.0;

extern StatesGroup g_lio_state;
extern MeasureGroup Measures;
class VoxelMapping
{
public:
    VoxelMapping(ros::NodeHandle &nh);

private:
    void Init(ros::NodeHandle &nh);
    void Run();
    void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg);
    void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in);
    bool sync_packages(MeasureGroup &meas);

private:
    void InitVoxelMap();

    void BodyVarAndCrossmatList(std::vector<M3D> &body_var, std::vector<M3D> &crossmat_list);

    void IESKFUpdate(const std::vector<M3D> &body_var, const std::vector<M3D> &crossmat_list);

    void ThreeSigmaCriterion(std::vector<ptpl> &ptpl_list, const std::vector<M3D> &body_var, const std::vector<M3D> &crossmat_list);

    void SetLaserCloudAndCorrNormvect(const std::vector<ptpl> &ptpl_list);

    void CalJacobianMatrix(const std::vector<ptpl> &ptpl_list, Eigen::MatrixXd &Hsub, Eigen::MatrixXd &Hsub_T_R_inv, Eigen::VectorXd &R_inv, Eigen::VectorXd &meas_vec);

    void UpdateState(const Eigen::MatrixXd &Hsub, const Eigen::MatrixXd &Hsub_T_R_inv, const Eigen::VectorXd &R_inv, const Eigen::VectorXd &meas_vec, Eigen::MatrixXd &K);

    bool ConvergenceJudgements(int iterCount, const Eigen::MatrixXd &K, const Eigen::MatrixXd &Hsub);

    void UpdateVoxelMap(const std::vector<M3D> &body_var, const std::vector<M3D> &crossmat_list);

    void pointBodyToWorld(PointType const *const pi, PointType *const po);

    template <typename T>
    void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
    {
        V3D p_body(pi[0], pi[1], pi[2]);
        p_body = p_body + Lidar_offset_to_IMU;
        V3D p_global(g_lio_state.rot_end * (p_body) + g_lio_state.pos_end);
        po[0] = p_global(0);
        po[1] = p_global(1);
        po[2] = p_global(2);
    }

    template <typename T>
    void set_posestamp(T &out)
    {
        out.position.x = g_lio_state.pos_end(0);
        out.position.y = g_lio_state.pos_end(1);
        out.position.z = g_lio_state.pos_end(2);
        out.orientation.x = geoQuat.x;
        out.orientation.y = geoQuat.y;
        out.orientation.z = geoQuat.z;
        out.orientation.w = geoQuat.w;
    }

    void RGBpointBodyToWorld(PointType const *const pi, PointType *const po);

    // void SigHandle(int sig);

    static inline bool var_contrast(pointWithCov &x, pointWithCov &y)
    {
        return (x.cov.diagonal().norm() < y.cov.diagonal().norm());
    };

private:
    void Publish();

    void publish_frame_world(const ros::Publisher &pubLaserCloudFullRes,
                             const int point_skip);
    void publish_effect_world(const ros::Publisher &pubLaserCloudEffect,
                              const ros::Publisher &pubPointWithCov,
                              const std::vector<ptpl> &ptpl_list);
    void publish_no_effect(const ros::Publisher &pubLaserCloudNoEffect);
    void publish_effect(const ros::Publisher &pubLaserCloudEffect);
    void publish_odometry(const ros::Publisher &pubOdomAftMapped);
    void publish_mavros(const ros::Publisher &mavros_pose_publisher);
    void publish_path(const ros::Publisher pubPath);

private:
    VD(DIM_STATE)
    solution;
    MD(DIM_STATE, DIM_STATE)
    G, H_T_H, I_STATE;
    V3D rot_add, t_add;
    StatesGroup state_propagat;

private:
    // MeasureGroup Measures;
    // StatesGroup g_lio_state;

    std::string lid_topic, imu_topic;
    double ranging_cov = 0.0;
    double angle_cov = 0.0;
    double gyr_cov_scale, acc_cov_scale;
    double imu_freq = 200.0;

    std::vector<double> extrinT;
    std::vector<double> extrinR;

    int NUM_MAX_ITERATIONS;
    int max_points_size = 50;
    int max_cov_points_size = 50;
    std::vector<double> layer_point_size;
    int max_layer = 0;
    double max_voxel_size = 1.0;
    double filter_size_surf_min;
    double min_eigen_value = 0.003;

    bool publish_voxel_map = false;
    int publish_max_voxel_layer = 0;
    bool publish_point_cloud = false;
    int pub_point_cloud_skip = 1;
    bool dense_map_en = true;

    int publish_count = 0;

    bool flg_exit = false;
    bool lidar_pushed = false;
    bool flg_reset = false;
    bool is_first_frame = true;
    bool init_map = false;

    bool EKF_stop_flg = false;
    bool flg_EKF_converged = false;
    bool nearest_search_en = true;

    int rematch_num = 0;

    bool flg_EKF_inited;
    double lidar_end_time = 0;
    double last_timestamp_imu = -1.0;
    double last_timestamp_lidar = -1.0;
    double first_lidar_time = 0;

    double total_residual = 0.0;
    int effct_feat_num = 0;

    PointCloudXYZI::Ptr feats_undistort;
    PointCloudXYZI::Ptr feats_down_body;
    PointCloudXYZI::Ptr laserCloudOri;
    PointCloudXYZI::Ptr corr_normvect;
    PointCloudXYZI::Ptr laserCloudNoeffect;

    geometry_msgs::Quaternion geoQuat;
    nav_msgs::Odometry odomAftMapped;
    geometry_msgs::PoseStamped msg_body_pose;

    V3D euler_cur;

    std::mutex mtx_buffer;
    std::condition_variable sig_buffer;

    std::vector<int> layer_size;

    std::deque<PointCloudXYZI::Ptr> lidar_buffer;
    std::deque<double> time_buffer;
    std::deque<sensor_msgs::Imu::ConstPtr> imu_buffer;

    std::unordered_map<VOXEL_LOC, OctoTree *> voxel_map;

    nav_msgs::Path path;

    ros::Subscriber sub_pcl;
    ros::Subscriber sub_imu;
    ros::Publisher pubLaserCloudFullRes;
    ros::Publisher pubLaserCloudEffect;
    ros::Publisher pubOdomAftMapped;
    ros::Publisher pubPath;
    ros::Publisher voxel_map_pub;

    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> downSizeFilterMap;

    std::shared_ptr<Preprocess> p_pre;
    std::shared_ptr<ImuProcess> p_imu;

    std::thread process;
};

#endif