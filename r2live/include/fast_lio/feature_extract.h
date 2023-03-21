#ifndef _FEATURE_EXTRACT_H_
#define _FEATURE_EXTRACT_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include "r2live/parameter_server.h"

struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time; 
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (float, time, time)
)


using PointXYZIRT = VelodynePointXYZIRT;
using PointType = pcl::PointXYZINormal;

class FeatureExtract {
    public:
        FeatureExtract(ros::NodeHandle& nh);
    private:
        void Init();
        void VelodyneHandler(const sensor_msgs::PointCloud2::ConstPtr &msg);
        void pub_func(pcl::PointCloud<PointType> &pl, ros::Publisher pub, const ros::Time &ct);
    private:
        ros::Publisher pub_full_;
        ros::Publisher pub_corn_;
        ros::Publisher pub_surf_;

        ros::Subscriber sub_points_;

        int n_scans_;
        bool given_offset_time_;

};

#endif