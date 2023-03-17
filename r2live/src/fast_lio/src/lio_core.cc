#include "lio_core.h"

LioCore::LioCore(int num_max_iterations, 
                double maximum_pt_kdtree_dis,
                double planar_check_dis,
                double long_rang_pt_dis,
                double maximum_res_dis)
{
    Init();
    num_max_iterations_ = num_max_iterations;
    maximum_pt_kdtree_dis_ = maximum_pt_kdtree_dis;
    planar_check_dis_ = planar_check_dis;
    long_rang_pt_dis_ = long_rang_pt_dis;
    maximum_res_dis_ = maximum_res_dis;
}

void LioCore::Init()
{
    laser_cloud_ori_.reset(new PointCloudXYZI());
    coeff_sel_.reset(new PointCloudXYZI());
}

void LioCore::SetPointCloudMap(std::shared_ptr<PointCloudMap> point_cloud_map)
{
    point_cloud_map_ = point_cloud_map;
}

void LioCore::ReSetData(PointCloudXYZI::Ptr current_frame)
{
    int points_size = current_frame->points.size();
    point_selected_surf_.resize(points_size, true);
    point_search_ind_surf_.resize(points_size);
    nearest_points_.resize(points_size);

    coeff_sel_tmpt_.reset(new PointCloudXYZI(*current_frame));
    feats_down_updated_.reset(new PointCloudXYZI(*current_frame));
    res_last_.resize(points_size, 1000.0);
}

void LioCore::PointBodyToWorld(PointType const *const pi, PointType *const po)
{
    Eigen::Vector3d p_body(pi->x, pi->y, pi->z);
    Eigen::Vector3d p_global(g_lio_state.rot_end * (p_body + Lidar_offset_to_IMU) + g_lio_state.pos_end);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void LioCore::PCASolver(PointVector& points_near, double ori_pt_dis, 
                        double& maximum_pt_range, const PointType &pointSel_tmpt,
                        int index)
{
    cv::Mat matA0(NUM_MATCH_POINTS, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matB0(NUM_MATCH_POINTS, 1, CV_32F, cv::Scalar::all(-1));
    cv::Mat matX0(NUM_MATCH_POINTS, 1, CV_32F, cv::Scalar::all(0));

    for (int j = 0; j < NUM_MATCH_POINTS; j++)
    {
        matA0.at<float>(j, 0) = points_near[j].x;
        matA0.at<float>(j, 1) = points_near[j].y;
        matA0.at<float>(j, 2) = points_near[j].z;
    }

    cv::solve(matA0, matB0, matX0, cv::DECOMP_QR);

    float pa = matX0.at<float>(0, 0);
    float pb = matX0.at<float>(1, 0);
    float pc = matX0.at<float>(2, 0);
    float pd = 1;

    //ps is the norm of the plane norm_vec vector
    //pd is the distance from point to plane
    float ps = sqrt(pa * pa + pb * pb + pc * pc);
    pa /= ps;
    pb /= ps;
    pc /= ps;
    pd /= ps;

    bool planeValid = true;
    for (int j = 0; j < NUM_MATCH_POINTS; j++)
    {           
        // ANCHOR -  Planar check
        if (fabs(pa * points_near[j].x +
                    pb * points_near[j].y +
                    pc * points_near[j].z + pd) > planar_check_dis_) // Raw 0.0
        {
            // ANCHOR - Far distance pt processing
            if (ori_pt_dis < maximum_pt_range * 0.90 || (ori_pt_dis < long_rang_pt_dis_))
            {
                planeValid = false;
                point_selected_surf_[index] = false;
                break;
            }
        }
    }

    if (planeValid)
    {
        //loss fuction
        float pd2 = pa * pointSel_tmpt.x + pb * pointSel_tmpt.y + pc * pointSel_tmpt.z + pd;
        //if(fabs(pd2) > 0.1) continue;
        float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointSel_tmpt.x * pointSel_tmpt.x + pointSel_tmpt.y * pointSel_tmpt.y + pointSel_tmpt.z * pointSel_tmpt.z));
        // ANCHOR -  Point to plane distance
        //if ((s > 0.80)) // && ((std::abs(pd2) - res_last[i]) < 3 * res_mean_last)) // Ori: 0.90
        double acc_distance = (ori_pt_dis < long_rang_pt_dis_) ? maximum_res_dis_: 1.0;
        //double acc_distance = m_maximum_res_dis;
        if(pd2 < acc_distance)
        {
            point_selected_surf_[index] = true;
            coeff_sel_tmpt_->points[index].x = pa;
            coeff_sel_tmpt_->points[index].y = pb;
            coeff_sel_tmpt_->points[index].z = pc;
            coeff_sel_tmpt_->points[index].intensity = pd2;
            res_last_[index] = std::abs(pd2);                               
        }
        else
        {
            point_selected_surf_[index] = false;
        }
    }
}


void LioCore::Update(PointCloudXYZI::Ptr current_frame)
{
    ReSetData(current_frame);

    int points_size = current_frame->points.size();
    double maximum_pt_range = 0.0;
    bool rematch_en = false;
    for (int iterCount = 0; iterCount < num_max_iterations_; iterCount++)
    {
        laser_cloud_ori_->clear();
        coeff_sel_->clear();

        for (int i = 0; i < points_size; i++)
        {
            PointType &pointOri_tmpt = current_frame->points[i];
            double ori_pt_dis = sqrt(pointOri_tmpt.x * pointOri_tmpt.x + pointOri_tmpt.y * pointOri_tmpt.y + pointOri_tmpt.z * pointOri_tmpt.z);
            maximum_pt_range = std::max(ori_pt_dis, maximum_pt_range);
            PointType &pointSel_tmpt = feats_down_updated_->points[i];
           
            PointBodyToWorld(&pointOri_tmpt, &pointSel_tmpt);
            std::vector<float> pointSearchSqDis_surf;
            auto &points_near = nearest_points_[i];

            if (iterCount == 0 || rematch_en)
            {
                point_selected_surf_[i] = true;
                point_cloud_map_->NearestSearch(pointSel_tmpt, NUM_MATCH_POINTS, points_near, pointSearchSqDis_surf);

                float max_distance = pointSearchSqDis_surf[NUM_MATCH_POINTS - 1];
                
                if (max_distance > maximum_pt_kdtree_dis_)
                {
                    point_selected_surf_[i] = false;
                }
            }

            if (point_selected_surf_[i] == false) continue;

            PCASolver(points_near, ori_pt_dis, maximum_pt_range, pointSel_tmpt, i);
        }


        /*****************下面封装求解雅克比矩阵**************/
        double total_residual = 0.0;
        laser_cloud_sel_num_ = 0;

        for (int i = 0; i < coeff_sel_tmpt_->points.size(); i++)
        {
            if (point_selected_surf_[i] && (res_last_[i] <= 2.0))
            {
                laser_cloud_ori_->push_back(current_frame->points[i]);
                coeff_sel_->push_back(coeff_sel_tmpt_->points[i]);
                total_residual += res_last_[i];
                laser_cloud_sel_num_++;
            }
        }

        res_mean_last_ = total_residual / laser_cloud_sel_num_;
    }
}