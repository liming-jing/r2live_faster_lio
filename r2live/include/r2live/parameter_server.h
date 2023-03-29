#ifndef _PARAMETER_SERVER_H_
#define _PARAMETER_SERVER_H_

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <string>
#include <mutex>
#include "common_lib.h"
#include <glog/logging.h>

class ParameterServer {
    private:
        ParameterServer(){}
        ~ParameterServer(){}

        ParameterServer(const ParameterServer& instance) = delete;
        const ParameterServer &operator=(const ParameterServer& instance) = delete;

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
        inline double GetLidarTimeDelay() const {return lidar_time_delay_;}
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
        inline bool GetFlagEKFInited() const {return flg_EKF_inited_;}

        inline int GetMapMethod() const {return map_method_;}

        /* feature extract get parameter */
        inline int GetNScans() const {return n_scans_;}
        
        /*ivox map parameters*/
        inline double GetIvoxGridResolution() const {return ivox_grid_resolution_;}
        inline int GetIvoxNearbyType() const {return ivox_nearby_type_;}

        /* imu process parameters */
        inline int GetMaxInitCount() const {return max_init_count_;}


        /* shared parameters */
        inline int GetNumMatchPoints() const {return num_match_points_;}


        /* vio parameters */
        inline double GetLidarDragCamTim() const {return lidar_drag_cam_tim_;}
        inline bool GetIfAccMulG() const {return if_acc_mul_G_;}
        inline bool GetIfLidarStartFirst() const {return if_lidar_start_first_;}
        inline bool GetIfWriteRes2Bag() const {return if_write_res_2_bag_;}
        inline bool GetIfDumpLog() const {return if_dump_log_;}


        /* voxel map parameters */
        inline double GetMaxVoxelSize() const {return max_voxel_size_;}
        inline int GetMaxLayer() const {return max_layer_;}
        inline std::vector<int>& GetLayerPointSize() {return layer_point_size_;}
        inline int GetMaxPointsSize() const {return max_points_size_;}
        inline double GetMinEigenValue() const {return min_eigen_value_;}
        inline double GetRangingCov() const {return ranging_cov_;}
        inline double GetAngleCov() const {return angle_cov_;}

    public:
        /*  set parameters */
        inline void SetFlagEKFInited(bool flg) {flg_EKF_inited_ = flg;}


    private:
        /* fast lio parameter */
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
        bool flg_EKF_inited_;

        int map_method_;
       
        /* feature extract parameter */
        int n_scans_;

        /* pointcloud ivox map parameters*/
        double ivox_grid_resolution_;
        int ivox_nearby_type_;

        /* imu process parameters */
        // int kMaxInitCount
        int max_init_count_;

        /* share parameters */
        int num_match_points_;

        /* vio parameters */

        double lidar_drag_cam_tim_;
        bool if_acc_mul_G_;
        bool if_lidar_start_first_;
        bool if_write_res_2_bag_;
        bool if_dump_log_;
        // std::string bag_file_name_;

        /* voxel map parameters */
        double max_voxel_size_;
        int max_layer_; 
        std::vector<int> layer_point_size_;
        int max_points_size_;
        double min_eigen_value_;
        double ranging_cov_;
        double angle_cov_;
};

#endif