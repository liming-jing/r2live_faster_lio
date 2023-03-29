#ifndef _VOXEL_MAP_UTIL_H_
#define _VOXEL_MAP_UTIL_H_

#include "omp.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <execution>
#include <openssl/md5.h>
#include <pcl/common/io.h>
#include <rosbag/bag.h>
#include <stdio.h>
#include <string>
#include <unordered_map>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/ros.h>
#include <mutex>

#define HASH_P 116101
#define MAX_N 10000000000

static int plane_id = 0;

// a point to plane matching structure
typedef struct ptpl {
  Eigen::Vector3d point;
  Eigen::Vector3d normal;
  Eigen::Vector3d center;
  Eigen::Matrix<double, 6, 6> plane_cov;
  double d;
  int layer;
} ptpl;

// 3D point with covariance
typedef struct pointWithCov {
  Eigen::Vector3d point;
  Eigen::Vector3d point_world;
  Eigen::Matrix3d cov;
} pointWithCov;

typedef struct Plane {
  Eigen::Vector3d center;
  Eigen::Vector3d normal;
  Eigen::Vector3d y_normal;
  Eigen::Vector3d x_normal;
  Eigen::Matrix3d covariance;
  Eigen::Matrix<double, 6, 6> plane_cov;
  float radius = 0;
  float min_eigen_value = 1;
  float mid_eigen_value = 1;
  float max_eigen_value = 1;
  float d = 0;
  int points_size = 0;

  bool is_plane = false;
  bool is_init = false;
  int id;
  // is_update and last_update_points_size are only for publish plane
  bool is_update = false;
  int last_update_points_size = 0;
  bool update_enable = true;
} Plane;

class VOXEL_LOC {
public:
  int64_t x, y, z;

  VOXEL_LOC(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0)
      : x(vx), y(vy), z(vz) {}

  bool operator==(const VOXEL_LOC &other) const {
    return (x == other.x && y == other.y && z == other.z);
  }
};

// Hash value
namespace std {
template <> struct hash<VOXEL_LOC> {
  int64_t operator()(const VOXEL_LOC &s) const {
    using std::hash;
    using std::size_t;
    return ((((s.z) * HASH_P) % MAX_N + (s.y)) * HASH_P) % MAX_N + (s.x);
  }
};
} // namespace std

class OctoTree {
public:
  std::vector<pointWithCov> temp_points_; // all points in an octo tree
  std::vector<pointWithCov> new_points_;  // new points in an octo tree
  Plane *plane_ptr_;
  int max_layer_;
  bool indoor_mode_;
  int layer_;
  int octo_state_; // 0 is end of tree, 1 is not
  OctoTree *leaves_[8];
  double voxel_center_[3]; // x, y, z
  std::vector<int> layer_point_size_;
  float quater_length_;
  float planer_threshold_;
  int max_plane_update_threshold_;
  int update_size_threshold_;
  int all_points_num_;
  int new_points_num_;
  int max_points_size_;
  int max_cov_points_size_;
  bool init_octo_;
  bool update_cov_enable_;
  bool update_enable_;
  OctoTree(int max_layer, int layer, std::vector<int> layer_point_size,
           int max_point_size, int max_cov_points_size, float planer_threshold);
  // check is plane , calc plane parameters including plane covariance
  void init_plane(const std::vector<pointWithCov> &points, Plane *plane);

  // only updaye plane normal, center and radius with new points
  void update_plane(const std::vector<pointWithCov> &points, Plane *plane); 

  void init_octo_tree();

  void cut_octo_tree();

  void UpdateOctoTree(const pointWithCov &pv);
};

void mapJet(double v, double vmin, double vmax, uint8_t &r, uint8_t &g,
            uint8_t &b) ;

void buildVoxelMap(const std::vector<pointWithCov> &input_points,
                   const float voxel_size, const int max_layer,
                   const std::vector<int> &layer_point_size,
                   const int max_points_size, const int max_cov_points_size,
                   const float planer_threshold,
                   std::unordered_map<VOXEL_LOC, OctoTree *> &feat_map);

void updateVoxelMap(const std::vector<pointWithCov> &input_points,
                    const float voxel_size, const int max_layer,
                    const std::vector<int> &layer_point_size,
                    const int max_points_size, const int max_cov_points_size,
                    const float planer_threshold,
                    std::unordered_map<VOXEL_LOC, OctoTree *> &feat_map);


void build_single_residual(const pointWithCov &pv, const OctoTree *current_octo,
                           const int current_layer, const int max_layer,
                           const double sigma_num, bool &is_sucess,
                           double &prob, ptpl &single_ptpl);

void GetUpdatePlane(const OctoTree *current_octo, const int pub_max_voxel_layer,
                    std::vector<Plane> &plane_list);


void BuildResidualListOMP(const std::unordered_map<VOXEL_LOC, OctoTree *> &voxel_map,
                          const double voxel_size, const double sigma_num,
                          const int max_layer,
                          const std::vector<pointWithCov> &pv_list,
                          std::vector<ptpl> &ptpl_list,
                          std::vector<Eigen::Vector3d> &non_match);

void BuildResidualListNormal(
    const std::unordered_map<VOXEL_LOC, OctoTree *> &voxel_map,
    const double voxel_size, const double sigma_num, const int max_layer,
    const std::vector<pointWithCov> &pv_list, std::vector<ptpl> &ptpl_list,
    std::vector<Eigen::Vector3d> &non_match) ;

void CalcVectQuation(const Eigen::Vector3d &x_vec, const Eigen::Vector3d &y_vec,
                     const Eigen::Vector3d &z_vec,
                     geometry_msgs::Quaternion &q) ;

void CalcQuation(const Eigen::Vector3d &vec, const int axis,
                 geometry_msgs::Quaternion &q);

void pubSinglePlane(visualization_msgs::MarkerArray &plane_pub,
                    const std::string plane_ns, const Plane &single_plane,
                    const float alpha, const Eigen::Vector3d rgb) ;

void pubNoPlaneMap(const std::unordered_map<VOXEL_LOC, OctoTree *> &feat_map,
                   const ros::Publisher &plane_map_pub) ;

void pubVoxelMap(const std::unordered_map<VOXEL_LOC, OctoTree *> &voxel_map,
                 const int pub_max_voxel_layer,
                 const ros::Publisher &plane_map_pub);

void pubPlaneMap(const std::unordered_map<VOXEL_LOC, OctoTree *> &feat_map,
                 const ros::Publisher &plane_map_pub) ;

void CalcBodyCov(Eigen::Vector3d &pb, const float range_inc,
                 const float degree_inc, Eigen::Matrix3d &cov);
#endif