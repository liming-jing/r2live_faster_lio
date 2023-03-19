#include <ros/ros.h>
#include <iostream>
#include "../r2live/src/fast_lio/include/parameter_server.h"

void test_parameter_server()
{
    ParameterServer* PS = ParameterServer::GetInstance();
    std::cout << "imu topic : " << PS->GetImuTopic() << std::endl;
    std::cout << "Cube len : " << PS->GetCubeLen() << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test");
    ros::NodeHandle nh;
    ParameterServer* para_server = ParameterServer::GetInstance();
    para_server->InitParamWithRos(nh);
    test_parameter_server();
    return 0;
}