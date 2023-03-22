#include <ros/ros.h>
#include <memory>

#include "r2live/r2live.h"
#include "r2live/parameter_server.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "r2live");
    ros::NodeHandle nh("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    ParameterServer* para_server = ParameterServer::GetInstance();
    para_server->InitParamWithRos(nh);

    std::shared_ptr<R2live> r2live_ptr = std::make_shared<R2live>(nh);
    
    ros::spin();
    return 0;
}