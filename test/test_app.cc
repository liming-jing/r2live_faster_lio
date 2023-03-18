#include "../r2live/src/fast_lio/include/parameter_server.h"

#include <iostream>

void test_parameter_server()
{
    ParameterServer* PS = ParameterServer::GetInstance();
    int num = 8;
    PS->SetNums(num);
    std::cout << "num : " << PS->GetNums() << std::endl;
    num = 10;
    PS->SetNums(num);
    std::cout << "num : " << PS->GetNums() << std::endl;
}

int main(int argc, char** argv)
{
    test_parameter_server();
    return 0;
}