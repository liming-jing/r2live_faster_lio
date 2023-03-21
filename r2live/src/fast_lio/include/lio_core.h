#ifndef _LIO_CORE_H_
#define _LIO_CORE_H_

#include <vector>
#include <memory>
#include <glog/logging.h>
#include "parameter_server.h"
#include "pointcloud_map.h"
#include "common_lib.h"

#define LASER_POINT_COV (0.00015)    

extern StatesGroup g_lio_state;

class LioCore{
    public:
        LioCore();

        void SetEKFFlg(bool flg_EKF_inited);
        void SetPointCloudMap(std::shared_ptr<PointCloudMap> point_cloud_map);
        void Update(PointCloudXYZI::Ptr current_frame);
        inline PointCloudXYZI::Ptr GetLaserCloudOri() {return laser_cloud_ori_;};
    private:
        void Init();
        void ReSetData(PointCloudXYZI::Ptr current_frame);
        void PointBodyToWorld(PointType const *const pi, PointType *const po);
        void PCASolver(PointVector& points_near, double ori_pt_dis,
                       double& maximum_pt_range, const PointType &pointSel_tmpt, int index);

        void CalculateJacobianMatrix(PointCloudXYZI::Ptr current_frame, Eigen::MatrixXd& Hsub, Eigen::VectorXd& meas_vec);
        void IEKFUpdateState(Eigen::MatrixXd& Hsub, Eigen::VectorXd& meas_vec);
        bool IEKFUpdateCovariance(Eigen::MatrixXd& Hsub);
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

    private:
        Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES> G, H_T_H, I_STATE;
        Eigen::MatrixXd K;
    private:
        std::shared_ptr<PointCloudMap> point_cloud_map_;

};

#endif