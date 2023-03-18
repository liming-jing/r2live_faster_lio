#ifndef _PARAMETER_SERVER_H_
#define _PARAMETER_SERVER_H_


class ParameterServer {
    private:
        ParameterServer(){};
        ~ParameterServer() {}

        static ParameterServer* instance ;

        class MemoryReclamation{
        public:
            ~MemoryReclamation()
            {
                if (ParameterServer::instance != nullptr)
                {
                    delete ParameterServer::instance;
                    ParameterServer::instance = nullptr;
                }
            }
        };

    public:
        static ParameterServer* GetInstance()
        {
            if (instance == nullptr)
            {
                instance = new ParameterServer();
                static MemoryReclamation mr;
            }
            return instance;
        }

    public:
        void SetNums(int num) {nums = num;}

    public:
        int GetNums() const {return nums;}
    private:
        int nums;
};

#endif