#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver/CustomMsg.h>
#include <pcl/filters/impl/filter.hpp>
#include <r2live/cloud_info.h>
#include <stdio.h>
#include <stdarg.h>
#include <opencv/cv.h>
#include <vector>
#include <queue>

struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;  // 问题？
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (float, time, time)
)

// Feature will be updated in next version

#define LOG() {fprintf(stderr, "%s:\t func %s \n\t Line %d:\t", __FILE__,__func__, __LINE__);\
fprintf(stderr, "\n");}

const float ang_res_y = 2.0;
const float ang_bottom = 15.0+0.1;
const float ang_res_x = 0.2;
const int Horizon_SCAN = 1800;
const float sensorMountAngle = 0.0;
const int groundScanInd = 7;
const float sensorMinimumRange = 1.0;
const float segmentAlphaX = ang_res_x / 180.0 * M_PI;
const float segmentAlphaY = ang_res_y / 180.0 * M_PI;
const float segmentTheta = 60.0/180.0*M_PI; // decrese this value may improve accuracy
const int segmentValidPointNum = 5;
const int segmentValidLineNum = 3;
const float edgeThreshold = 0.1;
const float surfThreshold = 0.1;

bool given_offset_time = false;

std_msgs::Header cloudHeader;

int labelCount;
uint16_t *queueIndX; // array for breadth-first search process of segmentation, for speed
uint16_t *queueIndY;

uint16_t *allPushedIndX; // array for tracking points of a segmented object
uint16_t *allPushedIndY;

float *cloudCurvature;
int *cloudNeighborPicked;
int *cloudLabel;

struct smoothness_t{ 
    float value;
    size_t ind;
};

struct by_value{ 
    bool operator()(smoothness_t const &left, smoothness_t const &right) { 
        return left.value < right.value;
    }
};

std::vector<smoothness_t> cloudSmoothness;

std::vector<std::pair<int8_t, int8_t> > neighborIterator; // neighbor iterator for segmentaiton process

r2live::cloud_info segMsg;
cv::Mat rangeMat;  
cv::Mat labelMat; // label matrix for segmentaiton marking
cv::Mat groundMat; // ground matrix for ground cloud marking


using namespace std;

#define IS_VALID(a)  ((abs(a)>1e8) ? true : false)

using PointXYZIRT = VelodynePointXYZIRT;
typedef pcl::PointXYZINormal PointType;
// typedef pcl::PointXYZI PointType;

pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;

pcl::PointCloud<PointType>::Ptr fullCloud; // projected velodyne raw cloud, but saved in the form of 1-D matrix
pcl::PointCloud<PointType>::Ptr fullInfoCloud; // same as fullCloud, but with intensity - range
pcl::PointCloud<PointType>::Ptr segmentedCloud;
pcl::PointCloud<PointType>::Ptr outlierCloud;


// velodyne 特征提取 并发送出去这些点
pcl::PointCloud<PointType>::Ptr point_cloud_orig;
pcl::PointCloud<PointType>::Ptr cornerPointsSharp; // 角点
pcl::PointCloud<PointType>::Ptr surfPointsFlat; // 平面

PointType nanPoint; // fill in fullCloud at each iteration

ros::Publisher pub_full, pub_surf, pub_corn;

enum LID_TYPE{MID, HORIZON, VELO16, OUST64};

enum Feature{Nor, Poss_Plane, Real_Plane, Edge_Jump, Edge_Plane, Wire, ZeroPoint};
enum Surround{Prev, Next};
enum E_jump{Nr_nor, Nr_zero, Nr_180, Nr_inf, Nr_blind};

struct orgtype
{
  double range;
  double dista; 
  double angle[2];
  double intersect;
  E_jump edj[2];
  Feature ftype;
  orgtype()
  {
    range = 0;
    edj[Prev] = Nr_nor;
    edj[Next] = Nr_nor;
    ftype = Nor;
    intersect = 2;
  }
};

const double rad2deg = 180*M_1_PI;

int lidar_type;
double blind, inf_bound;
int N_SCANS;
int group_size;
double disA, disB;
double limit_maxmid, limit_midmin, limit_maxmin;
double p2l_ratio;
double jump_up_limit, jump_down_limit;
double cos160;
double edgea, edgeb;
double smallp_intersect, smallp_ratio;
int point_filter_num;
int g_if_using_raw_point = 1;
int g_point_step = 3;
void mid_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
void horizon_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg);
void velo16_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
void oust64_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
void give_feature(pcl::PointCloud<PointType> &pl, vector<orgtype> &types, pcl::PointCloud<PointType> &pl_corn, pcl::PointCloud<PointType> &pl_surf);
void pub_func(pcl::PointCloud<PointType> &pl, ros::Publisher pub, const ros::Time &ct);
int plane_judge(const pcl::PointCloud<PointType> &pl, vector<orgtype> &types, uint i, uint &i_nex, Eigen::Vector3d &curr_direct);
bool small_plane(const pcl::PointCloud<PointType> &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct);
bool edge_jump_judge(const pcl::PointCloud<PointType> &pl, vector<orgtype> &types, uint i, Surround nor_dir);
void allocateMemory();
void copyPointCloud(const sensor_msgs::PointCloud2::ConstPtr& laserCloudMsg);
void findStartEndAngle();
void projectPointCloud();
void groundRemoval();
void cloudSegmentation();
void labelComponents(int row, int col);
void calculateSmoothness();
void markOccludedPoints();
void extractFeatures();
void resetParameters();
void ExtractFeatue();
void lego_loam_give_feature(const sensor_msgs::PointCloud2::ConstPtr &msg);
void VelodyneHandler(const sensor_msgs::PointCloud2::ConstPtr &msg);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "feature_extract");
  ros::NodeHandle n;

  n.param<int>("feature_extraction/lidar_type", lidar_type, 0);
  n.param<double>("feature_extraction/blind", blind, 0.01);
  n.param<double>("feature_extraction/inf_bound", inf_bound, 10);
  n.param<int>("feature_extraction/N_SCANS", N_SCANS, 1);
  n.param<int>("feature_extraction/group_size", group_size, 8);
  n.param<double>("feature_extraction/disA", disA, 0.01);
  n.param<double>("feature_extraction/disB", disB, 0.1);
  n.param<double>("feature_extraction/p2l_ratio", p2l_ratio, 400);
  n.param<double>("feature_extraction/limit_maxmid", limit_maxmid, 9);
  n.param<double>("feature_extraction/limit_midmin", limit_midmin, 16);
  n.param<double>("feature_extraction/limit_maxmin", limit_maxmin, 3.24);
  n.param<double>("feature_extraction/jump_up_limit", jump_up_limit, 175.0);
  n.param<double>("feature_extraction/jump_down_limit", jump_down_limit, 5.0);
  n.param<double>("feature_extraction/cos160", cos160, 160.0);
  n.param<double>("feature_extraction/edgea", edgea, 3);
  n.param<double>("feature_extraction/edgeb", edgeb, 0.05);
  n.param<double>("feature_extraction/smallp_intersect", smallp_intersect, 170.0);
  n.param<double>("feature_extraction/smallp_ratio", smallp_ratio, 1.2);
  n.param<int>("feature_extraction/point_filter_num", point_filter_num, 4);
  n.param<int>("feature_extraction/point_step", g_point_step, 3);
  n.param<int>("feature_extraction/using_raw_point", g_if_using_raw_point, 1);

  jump_up_limit = cos(jump_up_limit/180*M_PI);
  jump_down_limit = cos(jump_down_limit/180*M_PI);
  cos160 = cos(cos160/180*M_PI);
  smallp_intersect = cos(smallp_intersect/180*M_PI);


  // new add function 开辟内存
  allocateMemory();

  ros::Subscriber sub_points;
  
  switch(lidar_type)
  {
  case MID:
    printf("MID40\n");
    sub_points = n.subscribe("/livox/lidar", 1000, mid_handler,ros::TransportHints().tcpNoDelay());
    // sub_points = n.subscribe("/livox/lidar_1LVDG1S006J5GZ3", 1000, mid_handler);
    break;

  case HORIZON:
    printf("HORIZON\n");
    sub_points = n.subscribe("/livox/lidar", 1000, horizon_handler,ros::TransportHints().tcpNoDelay());
    break;

  case VELO16:
    printf("VELO32\n");
    // sub_points = n.subscribe("/velodyne_points", 1000, velo16_handler,ros::TransportHints().tcpNoDelay());
    // lego_loam_give_feature
    // sub_points = n.subscribe("/velodyne_points", 1000, lego_loam_give_feature,ros::TransportHints().tcpNoDelay());
    // VelodyneHandler
    sub_points = n.subscribe("/velodyne_points", 1000, VelodyneHandler,ros::TransportHints().tcpNoDelay());
    break;

  case OUST64:
    printf("OUST64\n");
    sub_points = n.subscribe("/os1_cloud_node/points", 1000, oust64_handler,ros::TransportHints().tcpNoDelay());
    break;
  
  default:
    printf("Lidar type is wrong.\n");
    exit(0);
    break;
  }

  pub_full = n.advertise<sensor_msgs::PointCloud2>("/laser_cloud", 100);
  pub_surf = n.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 100);
  pub_corn = n.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 100);

  ros::spin();
  return 0;
}


double vx, vy, vz;
void mid_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  pcl::PointCloud<PointType> pl;
  pcl::fromROSMsg(*msg, pl);

  pcl::PointCloud<PointType> pl_corn, pl_surf;
  vector<orgtype> types;
  uint plsize = pl.size()-1;
  pl_corn.reserve(plsize); pl_surf.reserve(plsize);
  types.resize(plsize+1);

  for(uint i=0; i<plsize; i++)
  {
    types[i].range = pl[i].x;
    vx = pl[i].x - pl[i+1].x;
    vy = pl[i].y - pl[i+1].y;
    vz = pl[i].z - pl[i+1].z;
    types[i].dista = vx*vx + vy*vy + vz*vz;
  }
  // plsize++;
  types[plsize].range = sqrt(pl[plsize].x*pl[plsize].x + pl[plsize].y*pl[plsize].y);

  give_feature(pl, types, pl_corn, pl_surf);

  ros::Time ct(ros::Time::now());
  pub_func(pl, pub_full, msg->header.stamp);
  pub_func(pl_surf, pub_surf, msg->header.stamp);
  pub_func(pl_corn, pub_corn, msg->header.stamp);
}

void horizon_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg)
{
  double t1 = omp_get_wtime();
  vector<pcl::PointCloud<PointType>> pl_buff(N_SCANS);
  vector<vector<orgtype>> typess(N_SCANS);
  pcl::PointCloud<PointType> pl_full, pl_corn, pl_surf;

  uint plsize = msg->point_num;

  pl_corn.reserve(plsize); pl_surf.reserve(plsize);
  pl_full.resize(plsize);

  for(int i=0; i<N_SCANS; i++)
  {
    pl_buff[i].reserve(plsize);
  }
  
  for(uint i=1; i<plsize; i++)
  {
    if((msg->points[i].line < N_SCANS) && ((msg->points[i].tag & 0x30) == 0x10)
        && (!IS_VALID(msg->points[i].x)) && (!IS_VALID(msg->points[i].y)) && (!IS_VALID(msg->points[i].z)))
    {
      pl_full[i].x = msg->points[i].x;
      pl_full[i].y = msg->points[i].y;
      pl_full[i].z = msg->points[i].z;
      pl_full[i].intensity = msg->points[i].reflectivity;
      pl_full[i].curvature = msg->points[i].offset_time / float(1000000); //use curvature as time of each laser points

    //  std::cout << "time: \t" << to_string(pl_full[i].curvature) << std::endl;

      if((std::abs(pl_full[i].x - pl_full[i-1].x) > 1e-7) 
          || (std::abs(pl_full[i].y - pl_full[i-1].y) > 1e-7)
          || (std::abs(pl_full[i].z - pl_full[i-1].z) > 1e-7))
      {
        pl_buff[msg->points[i].line].push_back(pl_full[i]);
      }
    }
  }

  if(pl_buff[0].size() <= 7) {return;}

  for(int j=0; j<N_SCANS; j++)
  {
    pcl::PointCloud<PointType> &pl = pl_buff[j];
    vector<orgtype> &types = typess[j];
    plsize = pl.size();
    types.resize(plsize);
    plsize--;
    for(uint i=0; i<plsize; i++)
    {

      types[i].range = pl[i].x*pl[i].x + pl[i].y*pl[i].y;
      vx = pl[i].x - pl[i+1].x;
      vy = pl[i].y - pl[i+1].y;
      vz = pl[i].z - pl[i+1].z;
      // std::cout<<vx<<" "<<vx<<" "<<vz<<" "<<std::endl;
    }
    // plsize++;
    types[plsize].range = pl[plsize].x*pl[plsize].x + pl[plsize].y*pl[plsize].y;

    give_feature(pl, types, pl_corn, pl_surf);
  }

  ros::Time ct;
  ct.fromNSec(msg->timebase);
  pub_func(pl_full, pub_full, msg->header.stamp);
  pub_func(pl_surf, pub_surf, msg->header.stamp);
  pub_func(pl_corn, pub_corn, msg->header.stamp);
  std::cout<<"[~~~~~~~~~~~~ Feature Extract ]: time: "<< omp_get_wtime() - t1<<" "<<msg->header.stamp.toSec()<<std::endl;
}

int orders[16] = {0, 2, 4, 6, 8, 10, 12, 14, 1, 3, 5, 7, 9, 11, 13, 15};

void velo16_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{

  pcl::PointCloud<PointXYZIRT>::Ptr cloud;
  cloud.reset(new pcl::PointCloud<PointXYZIRT>());

  sensor_msgs::PointCloud2 input_cloud = std::move(*msg);
  pcl::moveFromROSMsg(input_cloud, *cloud);
    // Remove Nan points
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

  pcl::PointCloud<PointType> pl_orig;
  // pcl::fromROSMsg(*msg, pl_orig);
  uint plsize = cloud->points.size();
  for (uint i = 0; i < plsize; i++)
  {
     PointType thisPoint;
     thisPoint.x = cloud->points[i].x;
     thisPoint.y = cloud->points[i].y;
     thisPoint.z = cloud->points[i].z;
     thisPoint.curvature = cloud->points[i].time;
     pl_orig.points.push_back(thisPoint);
  }

  vector<pcl::PointCloud<PointType>> pl_buff(N_SCANS);
  vector<vector<orgtype>> typess(N_SCANS);
  pcl::PointCloud<PointType> pl_corn, pl_surf, pl_full;

  int scanID;
  int last_stat = -1;
  int idx = 0;

  for(int i=0; i<N_SCANS; i++)
  {
    pl_buff[i].resize(plsize);
    typess[i].resize(plsize);
  }

  for(uint i=0; i<plsize; i++)
  {
    PointType &ap = pl_orig[i];
    double leng = sqrt(ap.x*ap.x + ap.y*ap.y);
    if(leng < blind)
    {
      continue;
    }

    double ang = atan(ap.z / leng)*rad2deg;
    scanID = int((ang + 15) / 2 + 0.5);

    if(scanID>=N_SCANS || scanID<0)
    {
      continue;
    }

    if(orders[scanID] <= last_stat)
    {
      idx++;
    }

    pl_buff[scanID][idx].x = ap.x;
    pl_buff[scanID][idx].y = ap.y;
    pl_buff[scanID][idx].z = ap.z;
    pl_buff[scanID][idx].intensity = ap.intensity;
    typess[scanID][idx].range = leng;
    last_stat = orders[scanID];
  }

  idx++;

  // LOG();
  for(int j=0; j<N_SCANS; j++)
  {
    pcl::PointCloud<PointType> &pl = pl_buff[j];
    vector<orgtype> &types = typess[j];
    pl.erase(pl.begin()+idx, pl.end());
    types.erase(types.begin()+idx, types.end());
    plsize = idx - 1;
    for(uint i=0; i<plsize; i++)
    {
      vx = pl[i].x - pl[i+1].x;
      vy = pl[i].y - pl[i+1].y;
      vz = pl[i].z - pl[i+1].z;
      types[i].dista = vx*vx + vy*vy + vz*vz;
    }

    types[plsize].range = sqrt(pl[plsize].x*pl[plsize].x + pl[plsize].y*pl[plsize].y);
    
    give_feature(pl, types, pl_corn, pl_surf);
  }
  // LOG();


  // 发布点云的面点和角点
  pub_func(pl_orig, pub_full, msg->header.stamp);
  // pub_func(pl_corn, pub_surf, msg->header.stamp);
  // pub_func(pl_surf, pub_corn, msg->header.stamp);
  pub_func(pl_surf, pub_surf, msg->header.stamp);
  pub_func(pl_corn, pub_corn, msg->header.stamp);
  // LOG();
}

void VelodyneHandler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
      const float time_scale = 1e-3;
      pcl::PointCloud<PointXYZIRT> pl_orig;
      pcl::fromROSMsg(*msg, pl_orig);

       pcl::PointCloud<PointType> cloud_out;
       int plsize = pl_orig.points.size();
      cloud_out.reserve(plsize);


      /*** These variables only works when no point timestamps given ***/
      double omega_l = 3.61;  // scan angular velocity
      std::vector<bool> is_first(N_SCANS, true);
      std::vector<double> yaw_fp(N_SCANS, 0.0);    // yaw of first scan point
      std::vector<float> yaw_last(N_SCANS, 0.0);   // yaw of last scan point
      std::vector<float> time_last(N_SCANS, 0.0);  // last offset time
      /*****************************************************************/
  

      if (pl_orig.points[plsize - 1].time > 0) {
          given_offset_time = true;
      } else {
          given_offset_time = false;
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

        if (!given_offset_time) {
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

    pub_func(cloud_out, pub_surf, msg->header.stamp);
}

// 功能发布：面点 角点 以及原始点云
void publishCloud()
{
    pub_func(*point_cloud_orig, pub_full, cloudHeader.stamp);
    pub_func(*cornerPointsSharp, pub_corn, cloudHeader.stamp);
    pub_func(*surfPointsFlat, pub_surf, cloudHeader.stamp);


    std::cout << "ldiar time: \t" << to_string(cloudHeader.stamp.toNSec()) << std::endl;
}

/**
 * 输入：velodyne 点云
 * 输出： 面点 pl_surf, 角点 pl_corn, （不带地面点）
*/

void lego_loam_give_feature(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    // 1. Convert ros message to pcl point cloud
    copyPointCloud(msg);
    // 2. Start and end angle of a scan
    findStartEndAngle();
    // 3. Range image projection
    projectPointCloud();
    // 4. Mark ground points
    groundRemoval();
    // 5. Point cloud segmentation
    cloudSegmentation();
    // 6. extract feature
    ExtractFeatue();
    // 6. pub cloud pl_surf pl_corn  pl_orig
    publishCloud();
    // 7. Reset parameters for next iteration
    resetParameters();
}

void copyPointCloud(const sensor_msgs::PointCloud2::ConstPtr& laserCloudMsg)
{
    cloudHeader.stamp = laserCloudMsg->header.stamp; // Ouster lidar users may need to uncomment this line
    sensor_msgs::PointCloud2 input_cloud = std::move(*laserCloudMsg);
    pcl::moveFromROSMsg(input_cloud, *laserCloudIn);
    // Remove Nan points
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*laserCloudIn, *laserCloudIn, indices); 
}

 void findStartEndAngle()
 {
    // start and end orientation of this cloud
    segMsg.startOrientation = -atan2(laserCloudIn->points[0].y, laserCloudIn->points[0].x);
    segMsg.endOrientation   = -atan2(laserCloudIn->points[laserCloudIn->points.size() - 1].y,
                                                    laserCloudIn->points[laserCloudIn->points.size() - 1].x) + 2 * M_PI;
    if (segMsg.endOrientation - segMsg.startOrientation > 3 * M_PI) {
        segMsg.endOrientation -= 2 * M_PI;
    } else if (segMsg.endOrientation - segMsg.startOrientation < M_PI)
        segMsg.endOrientation += 2 * M_PI;
    segMsg.orientationDiff = segMsg.endOrientation - segMsg.startOrientation;
}

 void projectPointCloud(){
    // range image projection
    float verticalAngle, horizonAngle, range;
    size_t rowIdn, columnIdn, index, cloudSize; 
    PointType thisPoint;
    cloudSize = laserCloudIn->points.size();
    for (size_t i = 0; i < cloudSize; ++i){

        thisPoint.x = laserCloudIn->points[i].x;
        thisPoint.y = laserCloudIn->points[i].y;
        thisPoint.z = laserCloudIn->points[i].z;
        // thisPoint.intensity = laserCloudIn->points[i].intensity;
        // thisPoint.curvature = laserCloudIn->points[i].time / float(1000000);
        thisPoint.curvature = laserCloudIn->points[i].time ;

        // std::cout << "time: \t" << to_string(laserCloudIn->points[i].time) << std::endl;

        // find the row and column index in the iamge for this point
        verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
            rowIdn = (verticalAngle + ang_bottom) / ang_res_y;
        if (rowIdn < 0 || rowIdn >= N_SCANS)
            continue;

        horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;

        columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
        if (columnIdn >= Horizon_SCAN)
            columnIdn -= Horizon_SCAN;

        if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
            continue;

        range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);
        if (range < sensorMinimumRange)
            continue;
        rangeMat.at<float>(rowIdn, columnIdn) = range;

        thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0;

        index = columnIdn  + rowIdn * Horizon_SCAN;

        fullCloud->points[index] = thisPoint;

        fullInfoCloud->points[index] = thisPoint;
        fullInfoCloud->points[index].intensity = range; // the corresponding range of a point is saved as "intensity"
        point_cloud_orig->points[index] = thisPoint;
    }
}

 void groundRemoval()
 {
        size_t lowerInd, upperInd;
        float diffX, diffY, diffZ, angle;

        int poinsSize = fullCloud->points.size();

        // groundMat
        // -1, no valid info to check if ground of not
        //  0, initial value, after validation, means not ground
        //  1, ground
        for (size_t j = 0; j < Horizon_SCAN; ++j){
            for (size_t i = 0; i < groundScanInd; ++i){

                lowerInd = j + ( i )*Horizon_SCAN;
                upperInd = j + (i+1)*Horizon_SCAN;

                if (lowerInd > poinsSize || upperInd > poinsSize) continue;
                
                if (fullCloud->points[lowerInd].intensity == -1 ||
                    fullCloud->points[upperInd].intensity == -1){
                    // no info to check, invalid points
                    groundMat.at<int8_t>(i,j) = -1;
                    continue;
                }
                // LOG();
                diffX = fullCloud->points[upperInd].x - fullCloud->points[lowerInd].x;
                diffY = fullCloud->points[upperInd].y - fullCloud->points[lowerInd].y;
                diffZ = fullCloud->points[upperInd].z - fullCloud->points[lowerInd].z;

                angle = atan2(diffZ, sqrt(diffX*diffX + diffY*diffY) ) * 180 / M_PI;

                // LOG();
                if (abs(angle - sensorMountAngle) <= 10){
                    groundMat.at<int8_t>(i,j) = 1;
                    groundMat.at<int8_t>(i+1,j) = 1;
                }
            }
        }

        // LOG();
        // extract ground cloud (groundMat == 1)
        // mark entry that doesn't need to label (ground and invalid point) for segmentation
        // note that ground remove is from 0~N_SCAN-1, need rangeMat for mark label matrix for the 16th scan
        for (size_t i = 0; i < N_SCANS; ++i){
            for (size_t j = 0; j < Horizon_SCAN; ++j){
                if (groundMat.at<int8_t>(i,j) == 1 || rangeMat.at<float>(i,j) == FLT_MAX){
                    labelMat.at<int>(i,j) = -1; // 标记地面点以及无效点
                }
            }
        }
        // LOG();

        /**
         *  TODO : 这里未将地面点去除！！！
        */

        // 原始代码 获取地面点云
        // if (pubGroundCloud.getNumSubscribers() != 0){
        //     for (size_t i = 0; i <= groundScanInd; ++i){
        //         for (size_t j = 0; j < Horizon_SCAN; ++j){
        //             if (groundMat.at<int8_t>(i,j) == 1)
        //                 groundCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
        //         }
        //     }
        // }
    }

void cloudSegmentation(){
    // segmentation process
    for (size_t i = 0; i < N_SCANS; ++i)
        for (size_t j = 0; j < Horizon_SCAN; ++j)
            if (labelMat.at<int>(i,j) == 0)
                labelComponents(i, j);

    int sizeOfSegCloud = 0;
    // extract segmented cloud for lidar odometry
    for (size_t i = 0; i < N_SCANS; ++i) {

        segMsg.startRingIndex[i] = sizeOfSegCloud-1 + 5;

        for (size_t j = 0; j < Horizon_SCAN; ++j) {
            if (labelMat.at<int>(i,j) > 0 || groundMat.at<int8_t>(i,j) == 1){
                // outliers that will not be used for optimization (always continue)
                if (labelMat.at<int>(i,j) == 999999){
                    if (i > groundScanInd && j % 5 == 0){
                        outlierCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                        continue;
                    }else{
                        continue;
                    }
                }
                // majority of ground points are skipped
                if (groundMat.at<int8_t>(i,j) == 1){
                    if (j%5!=0 && j>5 && j<Horizon_SCAN-5)
                        continue;
                }
                // mark ground points so they will not be considered as edge features later
                segMsg.segmentedCloudGroundFlag[sizeOfSegCloud] = (groundMat.at<int8_t>(i,j) == 1);
                // mark the points' column index for marking occlusion later
                segMsg.segmentedCloudColInd[sizeOfSegCloud] = j;
                // save range info
                segMsg.segmentedCloudRange[sizeOfSegCloud]  = rangeMat.at<float>(i,j);
                // save seg cloud
                segmentedCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                // size of seg cloud
                ++sizeOfSegCloud;
            }
        }

        segMsg.endRingIndex[i] = sizeOfSegCloud-1 - 5;
    }
    
    // extract segmented cloud for visualization
    // if (pubSegmentedCloudPure.getNumSubscribers() != 0){
    //     for (size_t i = 0; i < N_SCAN; ++i){
    //         for (size_t j = 0; j < Horizon_SCAN; ++j){
    //             if (labelMat.at<int>(i,j) > 0 && labelMat.at<int>(i,j) != 999999){
    //                 segmentedCloudPure->push_back(fullCloud->points[j + i*Horizon_SCAN]);
    //                 segmentedCloudPure->points.back().intensity = labelMat.at<int>(i,j);
    //             }
    //         }
    //     }
    // }
}


void labelComponents(int row, int col)
{
    // use std::queue std::vector std::deque will slow the program down greatly
    float d1, d2, alpha, angle;
    int fromIndX, fromIndY, thisIndX, thisIndY; 
    bool lineCountFlag[N_SCANS] = {false};

    queueIndX[0] = row;
    queueIndY[0] = col;
    int queueSize = 1;
    int queueStartInd = 0;
    int queueEndInd = 1;

    allPushedIndX[0] = row;
    allPushedIndY[0] = col;
    int allPushedIndSize = 1;
    
    while(queueSize > 0){
        // Pop point
        fromIndX = queueIndX[queueStartInd];
        fromIndY = queueIndY[queueStartInd];
        --queueSize;
        ++queueStartInd;
        // Mark popped point
        labelMat.at<int>(fromIndX, fromIndY) = labelCount;
        // Loop through all the neighboring grids of popped grid
        for (auto iter = neighborIterator.begin(); iter != neighborIterator.end(); ++iter){
            // new index
            thisIndX = fromIndX + (*iter).first;
            thisIndY = fromIndY + (*iter).second;
            // index should be within the boundary
            if (thisIndX < 0 || thisIndX >= N_SCANS)
                continue;
            // at range image margin (left or right side)
            if (thisIndY < 0)
                thisIndY = Horizon_SCAN - 1;
            if (thisIndY >= Horizon_SCAN)
                thisIndY = 0;
            // prevent infinite loop (caused by put already examined point back)
            if (labelMat.at<int>(thisIndX, thisIndY) != 0)
                continue;

            d1 = std::max(rangeMat.at<float>(fromIndX, fromIndY), 
                            rangeMat.at<float>(thisIndX, thisIndY));
            d2 = std::min(rangeMat.at<float>(fromIndX, fromIndY), 
                            rangeMat.at<float>(thisIndX, thisIndY));

            if ((*iter).first == 0)
                alpha = segmentAlphaX;
            else
                alpha = segmentAlphaY;

            angle = atan2(d2*sin(alpha), (d1 -d2*cos(alpha)));

            if (angle > segmentTheta){

                queueIndX[queueEndInd] = thisIndX;
                queueIndY[queueEndInd] = thisIndY;
                ++queueSize;
                ++queueEndInd;

                labelMat.at<int>(thisIndX, thisIndY) = labelCount;
                lineCountFlag[thisIndX] = true;

                allPushedIndX[allPushedIndSize] = thisIndX;
                allPushedIndY[allPushedIndSize] = thisIndY;
                ++allPushedIndSize;
            }
        }
    }

    // check if this segment is valid
    bool feasibleSegment = false;
    if (allPushedIndSize >= 30)
        feasibleSegment = true;
    else if (allPushedIndSize >= segmentValidPointNum){
        int lineCount = 0;
        for (size_t i = 0; i < N_SCANS; ++i)
            if (lineCountFlag[i] == true)
                ++lineCount;
        if (lineCount >= segmentValidLineNum)
            feasibleSegment = true;            
    }
    // segment is valid, mark these points
    if (feasibleSegment == true){
        ++labelCount;
    }else{ // segment is invalid, mark these points
        for (size_t i = 0; i < allPushedIndSize; ++i){
            labelMat.at<int>(allPushedIndX[i], allPushedIndY[i]) = 999999;
        }
    }
}

// 提取边缘特征和面特征
void ExtractFeatue()
{
    // 1. 计算平滑度
    calculateSmoothness();

    // 2. 标记遮挡点
    markOccludedPoints();

    // 3. 提取特征
    extractFeatures();
}


void calculateSmoothness()
{
    int cloudSize = segmentedCloud->points.size();
    for (int i = 5; i < cloudSize - 5; i++) {
        // 1. 计算相邻10个点深度差的和
        float diffRange = segMsg.segmentedCloudRange[i-5] + segMsg.segmentedCloudRange[i-4]
                        + segMsg.segmentedCloudRange[i-3] + segMsg.segmentedCloudRange[i-2]
                        + segMsg.segmentedCloudRange[i-1] - segMsg.segmentedCloudRange[i] * 10
                        + segMsg.segmentedCloudRange[i+1] + segMsg.segmentedCloudRange[i+2]
                        + segMsg.segmentedCloudRange[i+3] + segMsg.segmentedCloudRange[i+4]
                        + segMsg.segmentedCloudRange[i+5];            
        // 2. 取平方
        cloudCurvature[i] = diffRange*diffRange;

        cloudNeighborPicked[i] = 0;
        cloudLabel[i] = 0;
        // 3. 保存曲率，保存索引
        cloudSmoothness[i].value = cloudCurvature[i];
        cloudSmoothness[i].ind = i;
    }
}

void markOccludedPoints()
{
    int cloudSize = segmentedCloud->points.size();

    for (int i = 5; i < cloudSize - 6; ++i){

        float depth1 = segMsg.segmentedCloudRange[i];
        float depth2 = segMsg.segmentedCloudRange[i+1];
        int columnDiff = std::abs(int(segMsg.segmentedCloudColInd[i+1] - segMsg.segmentedCloudColInd[i]));
        // 1. 标记有遮挡的点
        if (columnDiff < 10){

            if (depth1 - depth2 > 0.3){
                cloudNeighborPicked[i - 5] = 1;
                cloudNeighborPicked[i - 4] = 1;
                cloudNeighborPicked[i - 3] = 1;
                cloudNeighborPicked[i - 2] = 1;
                cloudNeighborPicked[i - 1] = 1;
                cloudNeighborPicked[i] = 1;
            }else if (depth2 - depth1 > 0.3){
                cloudNeighborPicked[i + 1] = 1;
                cloudNeighborPicked[i + 2] = 1;
                cloudNeighborPicked[i + 3] = 1;
                cloudNeighborPicked[i + 4] = 1;
                cloudNeighborPicked[i + 5] = 1;
                cloudNeighborPicked[i + 6] = 1;
            }
        }

        float diff1 = std::abs(float(segMsg.segmentedCloudRange[i-1] - segMsg.segmentedCloudRange[i]));
        float diff2 = std::abs(float(segMsg.segmentedCloudRange[i+1] - segMsg.segmentedCloudRange[i]));
        // 2. 这里我理解是标记离群点，也就是和周围点相差比较大的点。
        if (diff1 > 0.02 * segMsg.segmentedCloudRange[i] && diff2 > 0.02 * segMsg.segmentedCloudRange[i])
            cloudNeighborPicked[i] = 1;
    }
}

void extractFeatures()
{
    cornerPointsSharp->clear();
    surfPointsFlat->clear();
    
    // 只分为两类特征点
    // cornerPointsLessSharp->clear();
    // surfPointsLessFlat->clear();

    for (int i = 0; i < N_SCANS; i++) {

        // surfPointsLessFlatScan->clear();
        // 1. 分为6个方向，每个方向分别选择线特征和面特征
        for (int j = 0; j < 6; j++) {
            int sp = (segMsg.startRingIndex[i] * (6 - j)    + segMsg.endRingIndex[i] * j) / 6;
            int ep = (segMsg.startRingIndex[i] * (5 - j)    + segMsg.endRingIndex[i] * (j + 1)) / 6 - 1;

            if (sp >= ep)
                continue;
            // 2. 根据曲率排序
            std::sort(cloudSmoothness.begin()+sp, cloudSmoothness.begin()+ep, by_value());

            int largestPickedNum = 0;
            // 3. 选择线特征，不为地面，segInfo.segmentedCloudGroundFlag[ind] == false
            for (int k = ep; k >= sp; k--) {
                int ind = cloudSmoothness[k].ind;
                if (cloudNeighborPicked[ind] == 0 &&
                    cloudCurvature[ind] > edgeThreshold &&
                    segMsg.segmentedCloudGroundFlag[ind] == false) {
                
                    largestPickedNum++;
                    // // 3.1 选择最多2个线特征
                    // if (largestPickedNum <= 2) {
                    //     cloudLabel[ind] = 2;
                    //     cornerPointsSharp->push_back(segmentedCloud->points[ind]);
                    //     // cornerPointsLessSharp->push_back(segmentedCloud->points[ind]);
                    // // 3.2 平滑一些的线特征20个，用于mapping
                    // } else if (largestPickedNum <= 20) {
                    //     cloudLabel[ind] = 1;
                    //     cornerPointsLessSharp->push_back(segmentedCloud->points[ind]);
                    // } else {
                    // // 3.3 超过则退出
                    //     break;
                    // }

                    if (largestPickedNum <= 20)
                    {
                        cloudLabel[ind] = 1;
                        cornerPointsSharp->push_back(segmentedCloud->points[ind]);
                    }
                    else {
                        break;
                    }

                    // 3.4 标记相邻点，防止特征点过于集中
                    cloudNeighborPicked[ind] = 1;
                    for (int l = 1; l <= 5; l++) {
                        int columnDiff = std::abs(int(segMsg.segmentedCloudColInd[ind + l] - segMsg.segmentedCloudColInd[ind + l - 1]));
                        if (columnDiff > 10)
                            break;
                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--) {
                        int columnDiff = std::abs(int(segMsg.segmentedCloudColInd[ind + l] - segMsg.segmentedCloudColInd[ind + l + 1]));
                        if (columnDiff > 10)
                            break;
                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }

            int smallestPickedNum = 0;
            // 4. 选择面特征，为地面，segInfo.segmentedCloudGroundFlag[ind] == true
            for (int k = sp; k <= ep; k++) {
                int ind = cloudSmoothness[k].ind;
                if (cloudNeighborPicked[ind] == 0 &&
                    cloudCurvature[ind] < surfThreshold &&
                    segMsg.segmentedCloudGroundFlag[ind] == true) {

                    cloudLabel[ind] = -1;
                    surfPointsFlat->push_back(segmentedCloud->points[ind]);
                    // 4.1 选择最多4个面特征
                    smallestPickedNum++;
                    if (smallestPickedNum >= 4) {
                        break;
                    }
                    // 4.2 标记相邻点
                    cloudNeighborPicked[ind] = 1;
                    for (int l = 1; l <= 5; l++) {

                        int columnDiff = std::abs(int(segMsg.segmentedCloudColInd[ind + l] - segMsg.segmentedCloudColInd[ind + l - 1]));
                        if (columnDiff > 10)
                            break;

                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--) {

                        int columnDiff = std::abs(int(segMsg.segmentedCloudColInd[ind + l] - segMsg.segmentedCloudColInd[ind + l + 1]));
                        if (columnDiff > 10)
                            break;

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }
            // 5. 选择是地面的面特征，和其它没被选择的点（除了地面的点，并且不是线特征）
            for (int k = sp; k <= ep; k++) {
                if (cloudLabel[k] <= 0) {
                    // surfPointsLessFlatScan->push_back(segmentedCloud->points[k]);
                    surfPointsFlat->push_back(segmentedCloud->points[k]);
                }
            }
        }
        // 5.1 下采样平滑一些的面特征
        // surfPointsLessFlatScanDS->clear();
        // downSizeFilter.setInputCloud(surfPointsLessFlatScan);
        // downSizeFilter.filter(*surfPointsLessFlatScanDS);

        // *surfPointsLessFlat += *surfPointsLessFlatScanDS;
    }
}

 void resetParameters(){
    laserCloudIn->clear();
    segmentedCloud->clear();
    outlierCloud->clear();
    surfPointsFlat->clear();
    cornerPointsSharp->clear();
    point_cloud_orig->clear();

    rangeMat = cv::Mat(N_SCANS, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
    groundMat = cv::Mat(N_SCANS, Horizon_SCAN, CV_8S, cv::Scalar::all(0));
    labelMat = cv::Mat(N_SCANS, Horizon_SCAN, CV_32S, cv::Scalar::all(0));
    labelCount = 1;

    std::fill(fullCloud->points.begin(), fullCloud->points.end(), nanPoint);
    std::fill(fullInfoCloud->points.begin(), fullInfoCloud->points.end(), nanPoint);
    std::fill(point_cloud_orig->points.begin(), point_cloud_orig->points.end(), nanPoint);
    }

 void allocateMemory(){

        laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());


        fullCloud.reset(new pcl::PointCloud<PointType>());
        fullInfoCloud.reset(new pcl::PointCloud<PointType>());
        segmentedCloud.reset(new pcl::PointCloud<PointType>());
        outlierCloud.reset(new pcl::PointCloud<PointType>());

        fullCloud->points.resize(N_SCANS*Horizon_SCAN);
        fullInfoCloud->points.resize(N_SCANS*Horizon_SCAN);

        segMsg.startRingIndex.assign(N_SCANS, 0);
        segMsg.endRingIndex.assign(N_SCANS, 0);

        segMsg.segmentedCloudGroundFlag.assign(N_SCANS*Horizon_SCAN, false);
        segMsg.segmentedCloudColInd.assign(N_SCANS*Horizon_SCAN, 0);
        segMsg.segmentedCloudRange.assign(N_SCANS*Horizon_SCAN, 0);

        std::pair<int8_t, int8_t> neighbor;
        neighbor.first = -1; neighbor.second =  0; neighborIterator.push_back(neighbor);
        neighbor.first =  0; neighbor.second =  1; neighborIterator.push_back(neighbor);
        neighbor.first =  0; neighbor.second = -1; neighborIterator.push_back(neighbor);
        neighbor.first =  1; neighbor.second =  0; neighborIterator.push_back(neighbor);

        allPushedIndX = new uint16_t[N_SCANS*Horizon_SCAN];
        allPushedIndY = new uint16_t[N_SCANS*Horizon_SCAN];

        queueIndX = new uint16_t[N_SCANS*Horizon_SCAN];
        queueIndY = new uint16_t[N_SCANS*Horizon_SCAN];

        cloudCurvature = new float[N_SCANS*Horizon_SCAN];
        cloudNeighborPicked = new int[N_SCANS*Horizon_SCAN];
        cloudLabel = new int[N_SCANS*Horizon_SCAN];

        cloudSmoothness.resize(N_SCANS*Horizon_SCAN);
        segmentedCloud.reset(new pcl::PointCloud<PointType>());
        outlierCloud.reset(new pcl::PointCloud<PointType>());

        cornerPointsSharp.reset(new pcl::PointCloud<PointType>());
        surfPointsFlat.reset(new pcl::PointCloud<PointType>());
        point_cloud_orig.reset(new pcl::PointCloud<PointType>());
        point_cloud_orig->points.resize(N_SCANS*Horizon_SCAN);

        resetParameters();
    }


void pub_func(pcl::PointCloud<PointType> &pl, ros::Publisher pub, const ros::Time &ct)
{
  pl.height = 1; pl.width = pl.size();
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(pl, output);
  output.header.frame_id = "livox";
  output.header.stamp = ct;
  pub.publish(output);
}



void velo16_handler1(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  pcl::PointCloud<PointType> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);

  pcl::PointCloud<PointType> pl_corn, pl_surf, pl_full;
  vector<pcl::PointCloud<PointType>> pl_buff(N_SCANS);
  vector<vector<orgtype>> typess(N_SCANS);

  uint plsize = pl_orig.size();
  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);
  pl_full.reserve(plsize);

  for(int i=0; i<N_SCANS; i++)
  {
    pl_buff[i].resize(plsize);
    typess[i].resize(plsize);
  }

  int idx = -1;
  int stat = 0; // 0代表上一次为0
  int scanID = 0;

  for(uint i=0; i<plsize; i++)
  {
    PointType &ap = pl_orig[i];
    double leng = sqrt(ap.x*ap.x + ap.y*ap.y);
    
    if(leng > blind)
    {
      if(stat == 0)
      {
        stat = 1;
        idx++;
      }

      double ang = atan(ap.z / leng)*rad2deg;
      scanID = int((ang + 15) / 2 + 0.5);
      if(scanID>=N_SCANS || scanID <0)
      {
        continue;
      }
      pl_buff[scanID][idx] = ap;
      typess[scanID][idx].range = leng;
      pl_full.push_back(ap);
    }
    else
    {
      stat = 0;
    }
  }

  // idx = 0;
  // int last_stat = -1;
  // for(uint i=0; i<plsize; i++)
  // {
  //   PointType &ap = pl_orig[i];
  //   // pl_full.push_back(ap);
  //   double leng = sqrt(ap.x*ap.x + ap.y*ap.y);
  //   if(leng < blind)
  //   {
  //     continue;
  //   }

  //   double ang = atan(ap.z / leng)*rad2deg;
  //   scanID = int((ang + 15) / 2 + 0.5);

  //   if(scanID>=N_SCANS || scanID<0)
  //   {
  //     continue;
  //   }

  //   if(orders[scanID] <= last_stat)
  //   {
  //     idx++;
  //   }

  //   pl_buff[scanID][idx].x = ap.x;
  //   pl_buff[scanID][idx].y = ap.y;
  //   pl_buff[scanID][idx].z = ap.z;
  //   pl_buff[scanID][idx].intensity = ap.intensity;

  //   last_stat = orders[scanID];
  // }

  idx++;

  for(int j=0; j<N_SCANS; j++)
  {
    pcl::PointCloud<PointType> &pl = pl_buff[j];
    vector<orgtype> &types = typess[j];
    pl.erase(pl.begin()+idx, pl.end());
    types.erase(types.begin()+idx, types.end());
    plsize = idx - 1;
    for(uint i=0; i<plsize; i++)
    {
      // types[i].range = sqrt(pl[i].x*pl[i].x + pl[i].y*pl[i].y);
      vx = pl[i].x - pl[i+1].x;
      vy = pl[i].y - pl[i+1].y;
      vz = pl[i].z - pl[i+1].z;
      types[i].dista = vx*vx + vy*vy + vz*vz;
    }
    types[plsize].range = sqrt(pl[plsize].x*pl[plsize].x + pl[plsize].y*pl[plsize].y);
    
    
    give_feature(pl, types, pl_corn, pl_surf);
  }

  pub_func(pl_full, pub_full, msg->header.stamp);
  pub_func(pl_surf, pub_surf, msg->header.stamp);
  pub_func(pl_corn, pub_corn, msg->header.stamp);
}

void oust64_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  pcl::PointCloud<PointType> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  
  vector<pcl::PointCloud<PointType>> pl_buff(N_SCANS);
  vector<vector<orgtype>> typess(N_SCANS);
  pcl::PointCloud<PointType> pl_corn, pl_surf;

  uint plsize = pl_orig.size();

  pl_corn.reserve(plsize); pl_surf.reserve(plsize);
  for(int i=0; i<N_SCANS; i++)
  {
    pl_buff[i].reserve(plsize);
  }

  for(uint i=0; i<plsize; i+=N_SCANS)
  {
    for(int j=0; j<N_SCANS; j++)
    {
      pl_buff[j].push_back(pl_orig[i+j]);
    }
  }

  for(int j=0; j<N_SCANS; j++)
  {
    pcl::PointCloud<PointType> &pl = pl_buff[j];
    vector<orgtype> &types = typess[j];
    plsize = pl.size() - 1;
    types.resize(plsize+1);
    for(uint i=0; i<plsize; i++)
    {
      types[i].range = pl[i].x*pl[i].x + pl[i].y*pl[i].y;
      vx = pl[i].x - pl[i+1].x;
      vy = pl[i].y - pl[i+1].y;
      vz = pl[i].z - pl[i+1].z;
    }
    types[plsize].range = pl[plsize].x*pl[plsize].x + pl[plsize].y*pl[plsize].y;
    give_feature(pl, types, pl_corn, pl_surf);
  }

  pub_func(pl_orig, pub_full, msg->header.stamp);
  pub_func(pl_surf, pub_surf, msg->header.stamp);
  pub_func(pl_corn, pub_corn, msg->header.stamp);
}


void give_feature(pcl::PointCloud<PointType> &pl, vector<orgtype> &types, pcl::PointCloud<PointType> &pl_corn, pcl::PointCloud<PointType> &pl_surf)
{
  uint plsize = pl.size();
  uint plsize2;
  if(plsize == 0)
  {
    printf("something wrong\n");
    return;
  }
  uint head = 0;

  while(types[head].range < blind)
  {
    head++;
  }

  // Surf
  plsize2 = (plsize > group_size) ? (plsize - group_size) : 0;

  Eigen::Vector3d curr_direct(Eigen::Vector3d::Zero());
  Eigen::Vector3d last_direct(Eigen::Vector3d::Zero());

  uint i_nex = 0, i2;
  uint last_i = 0; uint last_i_nex = 0;
  int last_state = 0;
  int plane_type;

  PointType ap;

  for(uint i=head; i<plsize2; i+=g_point_step)
  {
    if(types[i].range > blind)
    {
      ap.x = pl[i].x;
      ap.y = pl[i].y;
      ap.z = pl[i].z;
      ap.curvature = pl[i].curvature;
      pl_surf.push_back(ap);
    }
    if (g_if_using_raw_point)
    {
      continue;
    }
    // i_nex = i; 
    i2 = i;
    // std::cout<<" i: "<<i<<" i_nex "<<i_nex<<"group_size: "<<group_size<<" plsize "<<plsize<<" plsize2 "<<plsize2<<std::endl;
    plane_type = plane_judge(pl, types, i, i_nex, curr_direct);
    
    if(plane_type == 1)
    {
      for(uint j=i; j<=i_nex; j++)
      { 
        if(j!=i && j!=i_nex)
        {
          types[j].ftype = Real_Plane;
        }
        else
        {
          types[j].ftype = Poss_Plane;
        }
      }
      
      // if(last_state==1 && fabs(last_direct.sum())>0.5)
      if(last_state==1 && last_direct.norm()>0.1)
      {
        double mod = last_direct.transpose() * curr_direct;
        if(mod>-0.707 && mod<0.707)
        {
          types[i].ftype = Edge_Plane;
        }
        else
        {
          types[i].ftype = Real_Plane;
        }
      }
      
      i = i_nex - 1;
      last_state = 1;
    }
    else if(plane_type == 2)
    {
      i = i_nex;
      last_state = 0;
    }
    else if(plane_type == 0)
    {
      if(last_state == 1)
      {
        uint i_nex_tem;
        uint j;
        for(j=last_i+1; j<=last_i_nex; j++)
        {
          uint i_nex_tem2 = i_nex_tem;
          Eigen::Vector3d curr_direct2;

          uint ttem = plane_judge(pl, types, j, i_nex_tem, curr_direct2);

          if(ttem != 1)
          {
            i_nex_tem = i_nex_tem2;
            break;
          }
          curr_direct = curr_direct2;
        }

        if(j == last_i+1)
        {
          last_state = 0;
        }
        else
        {
          for(uint k=last_i_nex; k<=i_nex_tem; k++)
          {
            if(k != i_nex_tem)
            {
              types[k].ftype = Real_Plane;
            }
            else
            {
              types[k].ftype = Poss_Plane;
            }
          }
          i = i_nex_tem-1;
          i_nex = i_nex_tem;
          i2 = j-1;
          last_state = 1;
        }

      }
    }

    last_i = i2;
    last_i_nex = i_nex;
    last_direct = curr_direct;

  }
  if (g_if_using_raw_point)
  {
    return;
  }
  plsize2 = plsize > 3 ? plsize - 3 : 0;
  for(uint i=head+3; i<plsize2; i++)
  {
    if(types[i].range<blind || types[i].ftype>=Real_Plane)
    {
      continue;
    }

    if(types[i-1].dista<1e-16 || types[i].dista<1e-16)
    {
      continue;
    }

    Eigen::Vector3d vec_a(pl[i].x, pl[i].y, pl[i].z);
    Eigen::Vector3d vecs[2];

    for(int j=0; j<2; j++)
    {
      int m = -1;
      if(j == 1)
      {
        m = 1;
      }

      if(types[i+m].range < blind)
      {
        if(types[i].range > inf_bound)
        {
          types[i].edj[j] = Nr_inf;
        }
        else
        {
          types[i].edj[j] = Nr_blind;
        }
        continue;
      }

      vecs[j] = Eigen::Vector3d(pl[i+m].x, pl[i+m].y, pl[i+m].z);
      vecs[j] = vecs[j] - vec_a;
      
      types[i].angle[j] = vec_a.dot(vecs[j]) / vec_a.norm() / vecs[j].norm();
      if(types[i].angle[j] < jump_up_limit)
      {
        types[i].edj[j] = Nr_180;
      }
      else if(types[i].angle[j] > jump_down_limit)
      {
        types[i].edj[j] = Nr_zero;
      }
    }

    types[i].intersect = vecs[Prev].dot(vecs[Next]) / vecs[Prev].norm() / vecs[Next].norm();
    if(types[i].edj[Prev]==Nr_nor && types[i].edj[Next]==Nr_zero && types[i].dista>0.0225 && types[i].dista>4*types[i-1].dista)
    {
      if(types[i].intersect > cos160)
      {
        if(edge_jump_judge(pl, types, i, Prev))
        {
          types[i].ftype = Edge_Jump;
        }
      }
    }
    else if(types[i].edj[Prev]==Nr_zero && types[i].edj[Next]== Nr_nor && types[i-1].dista>0.0225 && types[i-1].dista>4*types[i].dista)
    {
      if(types[i].intersect > cos160)
      {
        if(edge_jump_judge(pl, types, i, Next))
        {
          types[i].ftype = Edge_Jump;
        }
      }
    }
    else if(types[i].edj[Prev]==Nr_nor && types[i].edj[Next]==Nr_inf)
    {
      if(edge_jump_judge(pl, types, i, Prev))
      {
        types[i].ftype = Edge_Jump;
      }
    }
    else if(types[i].edj[Prev]==Nr_inf && types[i].edj[Next]==Nr_nor)
    {
      if(edge_jump_judge(pl, types, i, Next))
      {
        types[i].ftype = Edge_Jump;
      }
     
    }
    else if(types[i].edj[Prev]>Nr_nor && types[i].edj[Next]>Nr_nor)
    {
      if(types[i].ftype == Nor)
      {
        types[i].ftype = Wire;
      }
    }
  }

  plsize2 = plsize-1;
  double ratio;
  for(uint i=head+1; i<plsize2; i++)
  {
    if(types[i].range<blind || types[i-1].range<blind || types[i+1].range<blind)
    {
      continue;
    }
    
    if(types[i-1].dista<1e-8 || types[i].dista<1e-8)
    {
      continue;
    }

    if(types[i].ftype == Nor)
    {
      if(types[i-1].dista > types[i].dista)
      {
        ratio = types[i-1].dista / types[i].dista;
      }
      else
      {
        ratio = types[i].dista / types[i-1].dista;
      }

      if(types[i].intersect<smallp_intersect && ratio < smallp_ratio)
      {
        if(types[i-1].ftype == Nor)
        {
          types[i-1].ftype = Real_Plane;
        }
        if(types[i+1].ftype == Nor)
        {
          types[i+1].ftype = Real_Plane;
        }
        types[i].ftype = Real_Plane;
      }
    }
  }

  int last_surface = -1;
  for(uint j=head; j<plsize; j++)
  {
    if(types[j].ftype==Poss_Plane || types[j].ftype==Real_Plane)
    {
      if(last_surface == -1)
      {
        last_surface = j;
      }
      
      if(j == uint(last_surface+point_filter_num-1))
      {

        PointType ap;
        ap.x = pl[j].x;
        ap.y = pl[j].y;
        ap.z = pl[j].z;
        ap.curvature = pl[j].curvature;
        pl_surf.push_back(ap);

        last_surface = -1;
      }
    }
    else
    {
      if(types[j].ftype==Edge_Jump || types[j].ftype==Edge_Plane)
      {
        pl_corn.push_back(pl[j]);
      }
      if(last_surface != -1)
      {
        PointType ap;
        for(uint k=last_surface; k<j; k++)
        {
          ap.x += pl[k].x;
          ap.y += pl[k].y;
          ap.z += pl[k].z;
          ap.curvature += pl[k].curvature;
        }
        ap.x /= (j-last_surface);
        ap.y /= (j-last_surface);
        ap.z /= (j-last_surface);
        ap.curvature /= (j-last_surface);
        pl_surf.push_back(ap);
      }
      last_surface = -1;
    }
  }
}

int plane_judge(const pcl::PointCloud<PointType> &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct)
{
  double group_dis = disA*types[i_cur].range + disB;
  group_dis = group_dis * group_dis;
  // i_nex = i_cur;

  double two_dis;
  vector<double> disarr;
  disarr.reserve(20);

  for(i_nex=i_cur; i_nex<i_cur+group_size; i_nex++)
  {
    if(types[i_nex].range < blind)
    {
      curr_direct.setZero();
      return 2;
    }
    disarr.push_back(types[i_nex].dista);
  }
  
  for(;;)
  {
    if((i_cur >= pl.size()) || (i_nex >= pl.size())) break;

    if(types[i_nex].range < blind)
    {
      curr_direct.setZero();
      return 2;
    }
    vx = pl[i_nex].x - pl[i_cur].x;
    vy = pl[i_nex].y - pl[i_cur].y;
    vz = pl[i_nex].z - pl[i_cur].z;
    two_dis = vx*vx + vy*vy + vz*vz;
    if(two_dis >= group_dis)
    {
      break;
    }
    disarr.push_back(types[i_nex].dista);
    i_nex++;
  }

  double leng_wid = 0;
  double v1[3], v2[3];
  for(uint j=i_cur+1; j<i_nex; j++)
  {
    if((j >= pl.size()) || (i_cur >= pl.size())) break;
    v1[0] = pl[j].x - pl[i_cur].x;
    v1[1] = pl[j].y - pl[i_cur].y;
    v1[2] = pl[j].z - pl[i_cur].z;

    v2[0] = v1[1]*vz - vy*v1[2];
    v2[1] = v1[2]*vx - v1[0]*vz;
    v2[2] = v1[0]*vy - vx*v1[1];

    double lw = v2[0]*v2[0] + v2[1]*v2[1] + v2[2]*v2[2];
    if(lw > leng_wid)
    {
      leng_wid = lw;
    }
  }


  if((two_dis*two_dis/leng_wid) < p2l_ratio)
  {
    curr_direct.setZero();
    return 0;
  }

  uint disarrsize = disarr.size();
  for(uint j=0; j<disarrsize-1; j++)
  {
    for(uint k=j+1; k<disarrsize; k++)
    {
      if(disarr[j] < disarr[k])
      {
        leng_wid = disarr[j];
        disarr[j] = disarr[k];
        disarr[k] = leng_wid;
      }
    }
  }

  if(disarr[disarr.size()-2] < 1e-16)
  {
    curr_direct.setZero();
    return 0;
  }

  if(lidar_type==MID || lidar_type==HORIZON)
  {
    double dismax_mid = disarr[0]/disarr[disarrsize/2];
    double dismid_min = disarr[disarrsize/2]/disarr[disarrsize-2];

    if(dismax_mid>=limit_maxmid || dismid_min>=limit_midmin)
    {
      curr_direct.setZero();
      return 0;
    }
  }
  else
  {
    double dismax_min = disarr[0] / disarr[disarrsize-2];
    if(dismax_min >= limit_maxmin)
    {
      curr_direct.setZero();
      return 0;
    }
  }
  
  curr_direct << vx, vy, vz;
  curr_direct.normalize();
  return 1;
}


bool edge_jump_judge(const pcl::PointCloud<PointType> &pl, vector<orgtype> &types, uint i, Surround nor_dir)
{
  if(nor_dir == 0)
  {
    if(types[i-1].range<blind || types[i-2].range<blind)
    {
      return false;
    }
  }
  else if(nor_dir == 1)
  {
    if(types[i+1].range<blind || types[i+2].range<blind)
    {
      return false;
    }
  }
  double d1 = types[i+nor_dir-1].dista;
  double d2 = types[i+3*nor_dir-2].dista;
  double d;

  if(d1<d2)
  {
    d = d1;
    d1 = d2;
    d2 = d;
  }

  d1 = sqrt(d1);
  d2 = sqrt(d2);

 
  if(d1>edgea*d2 || (d1-d2)>edgeb)
  {
    return false;
  }
  
  return true;
}


