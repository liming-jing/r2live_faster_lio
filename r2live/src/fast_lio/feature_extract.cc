#include "fast_lio/feature_extract.h"

FeatureExtract::FeatureExtract(ros::NodeHandle& nh)
{
    Init();
    
    pub_full_ = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud", 100);
    pub_surf_ = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 100);
    pub_corn_ = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 100);
    
    sub_points_ = nh.subscribe("/velodyne_points", 1000, &FeatureExtract::VelodyneHandler, this, ros::TransportHints().tcpNoDelay());
}

void FeatureExtract::Init()
{
    given_offset_time_ = false;
    ParameterServer* para_server = ParameterServer::GetInstance();
    n_scans_ = para_server->GetNScans();
}

void FeatureExtract::VelodyneHandler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
      const float time_scale = 1e-3;
      pcl::PointCloud<PointXYZIRT> pl_orig;
      pcl::fromROSMsg(*msg, pl_orig);

       pcl::PointCloud<PointType> cloud_out;
       int plsize = pl_orig.points.size();
      cloud_out.reserve(plsize);


      /*** These variables only works when no point timestamps given ***/
      double omega_l = 3.61;  // scan angular velocity
      std::vector<bool> is_first(n_scans_, true);
      std::vector<double> yaw_fp(n_scans_, 0.0);    // yaw of first scan point
      std::vector<float> yaw_last(n_scans_, 0.0);   // yaw of last scan point
      std::vector<float> time_last(n_scans_, 0.0);  // last offset time
      /*****************************************************************/
  

      if (pl_orig.points[plsize - 1].time > 0) {
          given_offset_time_ = true;
      } else {
          given_offset_time_ = false;
          double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578;
          double yaw_end = yaw_first;
          int layer_first = pl_orig.points[0].ring;
          for (uint i = plsize - 1; i > 0; i--) {
              if (pl_orig.points[i].ring == layer_first) {
                  yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;
                  break;
              }
          }
      }

       for (int i = 0; i < plsize; i++) {
        PointType added_pt;

        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;
        added_pt.curvature = pl_orig.points[i].time * time_scale;  // curvature unit: ms

        if (!given_offset_time_) {
            int layer = pl_orig.points[i].ring;
            double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

            if (is_first[layer]) {
                yaw_fp[layer] = yaw_angle;
                is_first[layer] = false;
                added_pt.curvature = 0.0;
                yaw_last[layer] = yaw_angle;
                time_last[layer] = added_pt.curvature;
                continue;
            }

            // compute offset time
            if (yaw_angle <= yaw_fp[layer]) {
                added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l;
            } else {
                added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
            }

            if (added_pt.curvature < time_last[layer]) added_pt.curvature += 360.0 / omega_l;

            yaw_last[layer] = yaw_angle;
            time_last[layer] = added_pt.curvature;
        }

            if (i % 3 == 0) {
            if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z > 16) {
                cloud_out.points.push_back(added_pt);
            }
        }
    }

    pub_func(cloud_out, pub_surf_, msg->header.stamp);
}

void FeatureExtract::pub_func(pcl::PointCloud<PointType> &pl, ros::Publisher pub, const ros::Time &ct)
{
    pl.height = 1; 
    pl.width = pl.size();
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(pl, output);
    output.header.frame_id = "livox";
    output.header.stamp = ct;
    pub.publish(output);
}

