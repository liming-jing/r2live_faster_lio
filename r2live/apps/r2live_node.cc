#include <ros/ros.h>
#include <memory>
#include <glog/logging.h>

#include "r2live/r2live.h"
#include "r2live/parameter_server.h"

int main(int argc, char** argv)
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = true;
    FLAGS_logtostderr = true;

    ros::init(argc, argv, "r2live");
    ros::NodeHandle nh("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    ParameterServer* para_server = ParameterServer::GetInstance();
    para_server->InitParamWithRos(nh);

    std::shared_ptr<R2live> r2live_ptr = std::make_shared<R2live>(nh);
    
    ros::spin();
    return 0;
}