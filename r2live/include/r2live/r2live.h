#ifndef _R2LIVE_H_
#define _R2LIVE_H_

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <thread>
#include <mutex>

#include "vins/estimator.h"
#include "vins/parameters.h"
#include "utility/visualization.h"
#include "fast_lio/fast_lio.h"
#include "fast_lio/r2live/parameter_server.h"

class R2live {
    public:
        R2live(ros::NodeHandle& nh);

    private:
        void Init();
        void predict(const sensor_msgs::ImuConstPtr &imu_msg);
        void update();

        std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> getMeasurements();

    private:
        void imu_callback(const sensor_msgs::ImuConstPtr &_imu_msg);
        void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg);
        void restart_callback(const std_msgs::BoolConstPtr &restart_msg);
};

#endif