#ifndef _LIO_CORE_H_
#define _LIO_CORE_H_

#include <vector>
#include <memory>
#include <glog/logging.h>
#include "r2live/parameter_server.h"
#include "pointcloud_map_base.h"
#include "r2live/common_lib.h"
#include "fast_lio/voxel_map/voxel_map.h"

extern StatesGroup g_lio_state;

class LioCore{
    public:
        LioCore();
        
        void SetMap(std::shared_ptr<PointCloudMapBase> map_base_ptr);
        void Update(PointCloudXYZI::Ptr current_frame);
        inline PointCloudXYZI::Ptr GetLaserCloudOri() {return laser_cloud_ori_;}
        std::vector<PointVector>& GetNearestPoints() {return nearest_points_;}
    private:
        void Init();

        void UpdateNoVoxelMap(PointCloudXYZI::Ptr current_frame);
        void UpdateVoxelMap(PointCloudXYZI::Ptr current_frame);

    private:
        /* updateNoVoxelMap */
        void ReSetData(PointCloudXYZI::Ptr current_frame);
        void PointBodyToWorld(PointType const *const pi, PointType *const po);
        void PCASolver(PointVector& points_near, double ori_pt_dis,
                       double& maximum_pt_range, const PointType &pointSel_tmpt, int index);

        void CalculateJacobianMatrix(PointCloudXYZI::Ptr current_frame, Eigen::MatrixXd& Hsub, Eigen::VectorXd& meas_vec);
        void IEKFUpdateState(Eigen::MatrixXd& Hsub, Eigen::VectorXd& meas_vec);
        bool IEKFUpdateCovariance(Eigen::MatrixXd& Hsub);

    private:
        /* update voxel map*/
        void CalcBodyCovAndCrossmat(PointCloudXYZI::Ptr current_frame, 
                                    std::vector<Eigen::Matrix3d>& body_var, 
                                    std::vector<Eigen::Matrix3d>& crossmat_list);
        void OrganizeData(PointCloudXYZI::Ptr current_frame, 
                          const std::vector<Eigen::Matrix3d>& body_var, 
                          const std::vector<Eigen::Matrix3d>& crossmat_list,
                          std::vector<pointWithCov>& pv_list);
        void CalculateJacobianMatrix(const std::vector<ptpl>& ptpl_list,
                                     Eigen::MatrixXd& Hsub,
                                     Eigen::MatrixXd& Hsub_T_R_inv,
                                     Eigen::VectorXd& R_inv,
                                     Eigen::VectorXd& meas_vec);

        void IEKFUpdateStateForVoxelMap(const std::vector<ptpl>& ptpl_list,
                                        const Eigen::MatrixXd& Hsub,
                                        const Eigen::MatrixXd& Hsub_T_R_inv,
                                        const Eigen::VectorXd& meas_vec);
        
        void TransformPointBody2World(const PointCloudXYZI::Ptr &point_cloud_body, 
                                      pcl::PointCloud<pcl::PointXYZI>::Ptr &point_cloud_world);

        void PointTypeBodyToWorld(PointType const *const pi, PointType *const po);
    private:
        double maximum_pt_kdtree_dis_ = 1.0;
        double planar_check_dis_ = 0.05;
        double long_rang_pt_dis_ = 50.0;
        double maximum_res_dis_ = 1.0;
        int num_max_iterations_ = 0;
        int laser_cloud_sel_num_ = 0;
        double res_mean_last_ = 0.05;
        bool flg_EKF_inited_ = false;
        bool flg_EKF_converged_ = false;
        double delta_R_ = 0.0, delta_T_ = 0.0;
        bool rematch_en_ = false;
        int rematch_num_ = 0;
        int iter_count_ = 0;
        int num_match_points_ = 0;

        PointCloudXYZI::Ptr laser_cloud_ori_;
        PointCloudXYZI::Ptr coeff_sel_;

        std::vector<bool> point_selected_surf_;
        std::vector<std::vector<int>> point_search_ind_surf_;
        std::vector<PointVector> nearest_points_;

        PointCloudXYZI::Ptr coeff_sel_tmpt_;
        PointCloudXYZI::Ptr feats_down_updated_;
        std::vector<double> res_last_; 

        // new parameter for UpdateVoxelMap()
        double ranging_cov_;
        double angle_cov_;
        double max_voxel_size_;
        int max_layer_; 
        int max_points_size_;
        double min_eigen_value_;

    private:
        Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES> G, H_T_H, I_STATE;
        Eigen::MatrixXd K;
    private:
        std::shared_ptr<PointCloudMapBase> map_base_ptr_;

};

#endif