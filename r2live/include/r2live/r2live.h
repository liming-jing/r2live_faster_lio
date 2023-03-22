#ifndef _R2LIVE_H_
#define _R2LIVE_H_

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>

#include "vins/estimator.h"
#include "vins/parameters.h"
#include "utility/visualization.h"
#include "fast_lio/fast_lio.h"
#include "r2live/parameter_server.h"

#define CAM_MEASUREMENT_COV 1e-3

class R2live {
    public:
        R2live(ros::NodeHandle& nh);

    private:
        void Init(ros::NodeHandle& nh);
        void Predict(const sensor_msgs::ImuConstPtr &imu_msg);
        void Update();
        void Process();

        std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> GetMeasurements();
    
    private:
        void UnLockLio(Estimator &estimator);
        void LockLio(Estimator &estimator);
        void SyncLio2Vio(Estimator &estimator);

        void VisualImuMeasure(const Eigen::Vector3d &pts_i, const Eigen::Vector3d &pts_j,
                        const Eigen::Vector3d &Pi, const Eigen::Quaterniond &Qi,
                        const Eigen::Vector3d &Pj, const Eigen::Quaterniond &Qj,
                        const Eigen::Vector3d &tic, const Eigen::Quaterniond &qic,
                        const double inv_dep_i, Eigen::Vector2d & residual,
                        Eigen::Matrix<double, 2, 6, Eigen::RowMajor> & j_mat);
        
        void ConstructCameraMeasure(int frame_idx, Estimator &estimator,
                              std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> &reppro_err_vec,
                              std::vector<Eigen::Matrix<double, 2, 6, Eigen::RowMajor>, Eigen::aligned_allocator<Eigen::Matrix<double, 2, 6, Eigen::RowMajor>>> &J_mat_vec);

    private:
        void ImuCallback(const sensor_msgs::ImuConstPtr &imu_msg);
        void FeatureCallback(const sensor_msgs::PointCloudConstPtr &feature_msg);
        void RestartCallback(const std_msgs::BoolConstPtr &restart_msg);
        void RelocalizationCallback(const sensor_msgs::PointCloudConstPtr &points_msg);

    private:
        double last_imu_t_;
        double current_time_;
        double latest_time_;
        bool init_feature_;
        bool init_imu_;
        int sum_of_wait_;
        std::mutex buf_mutex_;
        std::mutex state_mutex_;
        std::mutex estimator_mutex_;
        std::condition_variable con_;

        Eigen::Vector3d tmp_P_;
        Eigen::Quaterniond tmp_Q_;
        Eigen::Vector3d tmp_V_;
        Eigen::Vector3d tmp_Ba_;
        Eigen::Vector3d tmp_Bg_;
        Eigen::Vector3d acc_0_;
        Eigen::Vector3d gyr_0_;

        eigen_q diff_vins_lio_q_ ;
        vec_3 diff_vins_lio_t_;

        queue<sensor_msgs::ImuConstPtr> imu_buf_;
        queue<sensor_msgs::PointCloudConstPtr> feature_buf_;
        queue<sensor_msgs::PointCloudConstPtr> relo_buf_;
    private:
        Estimator estimator_;

        ros::Subscriber sub_imu_;
        ros::Subscriber sub_image_;
        ros::Subscriber sub_restart_;
        ros::Subscriber sub_relo_points_;

        std::thread measurement_process_;
};

#endif