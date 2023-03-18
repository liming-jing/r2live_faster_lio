#ifndef _PARAMETER_SERVER_H_
#define _PARAMETER_SERVER_H_

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <string>
#include "common_lib.h"

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

        void InitParamWithRos(ros::NodeHandle & nh);
        void InitParamWithoutRos(std::string yaml_file);

    public:
        /*fast_lio get parameter*/
        inline std::string GetImuTopic() const {return imu_topic_;}
        inline bool GetDenseMapEn() const {return dense_map_en_;}
        inline int GetNumMaxIterations() const {return num_max_iterations_;}
        inline double GetFovDeg() const {return fov_deg_;}
        inline double GetFilterSizeCornerMin() const {return filter_size_corner_min_;}
        inline double GetFilterSizeSurfMin() const {return filter_size_surf_min_;}
        inline double GetFilterSizeMapMin() const {return filter_size_map_min_;}
        inline double GetFilterSizeSurfMinZ() const {return filter_size_surf_min_z_;}
        inline double GetCubeLen() const {return cube_len_;}
        inline double GetMaximumPtKdtreeDis() const {return maximum_pt_kdtree_dis_;}
        inline double GetMaximumResDis() const {return maximum_res_dis_;}
        inline double GetPlanarCheckDis() const {return planar_check_dis_;}
        inline double GetLongRangPtDis() const {return long_rang_pt_dis_;}
        inline bool GetIfPublishFeatureMap() const {return if_publish_feature_map_;}

    private:
         /*fast_lio_parameter*/
        std::string imu_topic_;
        bool dense_map_en_;
        double lidar_time_delay_;
        int num_max_iterations_;
        double fov_deg_;
        double filter_size_corner_min_;
        double filter_size_surf_min_;
        double filter_size_map_min_;
        double filter_size_surf_min_z_;
        double cube_len_ = 0.0;
        double maximum_pt_kdtree_dis_;
        double maximum_res_dis_;
        double planar_check_dis_;
        double long_rang_pt_dis_;
        bool if_publish_feature_map_;


};

#endif