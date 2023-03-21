#include <ros/ros.h>
#include <memory>

#include "fast_lio/feature_extract.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lio_feature_extract_node");
    ros::NodeHandle nh;

    std::shared_ptr<FeatureExtract> feature_extract_ptr = std::make_shared<FeatureExtract>(nh);
    ros::Rate rate(5000);
    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}