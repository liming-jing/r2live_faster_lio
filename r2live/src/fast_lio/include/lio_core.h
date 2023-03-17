#ifndef _LIO_CORE_H_
#define _LIO_CORE_H_

#include <vector>
#include <memory>
#include "pointcloud_map.h"
#include "common_lib.h"

#define NUM_MATCH_POINTS (5)
extern StatesGroup g_lio_state;

class LioCore{
    public:
        LioCore(int num_max_iterations, 
        double maximum_pt_kdtree_dis,
        double planar_check_dis,
        double long_rang_pt_dis,
        double maximum_res_dis);

        void SetPointCloudMap(std::shared_ptr<PointCloudMap> point_cloud_map);
        void Update(PointCloudXYZI::Ptr current_frame);
    private:
        void Init();
        void ReSetData(PointCloudXYZI::Ptr current_frame);
        void PointBodyToWorld(PointType const *const pi, PointType *const po);
        void PCASolver(PointVector& points_near, double ori_pt_dis,
                       double& maximum_pt_range, const PointType &pointSel_tmpt, int index);

        // double GetResMeanLast()
    private:
        double maximum_pt_kdtree_dis_ = 1.0;
        double planar_check_dis_ = 0.05;
        double long_rang_pt_dis_ = 50.0;
        double maximum_res_dis_ = 1.0;
        int num_max_iterations_ = 0;
        int laser_cloud_sel_num_ = 0;
        double res_mean_last_ = 0.05;

        PointCloudXYZI::Ptr laser_cloud_ori_;
        PointCloudXYZI::Ptr coeff_sel_;

        std::vector<bool> point_selected_surf_;
        std::vector<std::vector<int>> point_search_ind_surf_;
        std::vector<PointVector> nearest_points_;

        PointCloudXYZI::Ptr coeff_sel_tmpt_;
        PointCloudXYZI::Ptr feats_down_updated_;
        std::vector<double> res_last_; 

    private:
        std::shared_ptr<PointCloudMap> point_cloud_map_;

};

#endif