#ifndef COMMON_LIB_H
#define COMMON_LIB_H

#include <Eigen/Eigen>
#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/Imu.h>
#include <so3_math.h>
#include <tf/transform_broadcaster.h>
#include <r2live/Pose6D.h>
#include <r2live/States.h>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/PointCloud2.h>

#include "tools_color_printf.hpp"

using namespace std;
using namespace Eigen;

#define PI_M (3.14159265358)
#define G_m_s2 (9.81)  // Gravaty const in GuangDong/China
#define DIM_STATE (18) // Dimension of states (Let Dim(SO(3)) = 3)
#define DIM_OF_STATES (18)

#define DIM_PROC_N (12) // Dimension of process noise (Let Dim(SO(3)) = 3)
#define CUBE_LEN (6.0)
#define LIDAR_SP_LEN (2)
// old init
#define INIT_COV (0.0000001)
#define NUM_MATCH_POINTS (5)
#define MAX_MEAS_DIM (10000)

#define VEC_FROM_ARRAY(v) v[0], v[1], v[2]
#define MAT_FROM_ARRAY(v) v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]
#define CONSTRAIN(v, min, max) ((v > min) ? ((v < max) ? v : max) : min)
#define ARRAY_FROM_EIGEN(mat) mat.data(), mat.data() + mat.rows() * mat.cols()
#define STD_VEC_FROM_EIGEN(mat)               \
    vector<decltype(mat)::Scalar>(mat.data(), \
                                  mat.data() + mat.rows() * mat.cols())
#define DEBUG_FILE_DIR(name) (string(string(ROOT_DIR) + "Log/" + name))

typedef r2live::Pose6D Pose6D;
typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;
typedef vector<PointType, Eigen::aligned_allocator<PointType>> PointVector;
typedef Vector3d V3D;
typedef Matrix3d M3D;
typedef Vector3f V3F;
typedef Matrix3f M3F;

#define MD(a, b) Matrix<double, (a), (b)>
#define VD(a) Matrix<double, (a), 1>
#define MF(a, b) Matrix<float, (a), (b)>
#define VF(a) Matrix<float, (a), 1>

static const Eigen::Matrix3d Eye3d(Eigen::Matrix3d::Identity());
static const Eigen::Matrix3f Eye3f(Eigen::Matrix3f::Identity());
static const Eigen::Vector3d Zero3d(0, 0, 0);
static const Eigen::Vector3f Zero3f(0, 0, 0);
static const Eigen::Vector3d Lidar_offset_to_IMU(0, 0, 0);

template <typename T>
T get_ros_parameter(ros::NodeHandle &nh, const std::string parameter_name, T &parameter, T default_val)
{
    nh.param<T>(parameter_name.c_str(), parameter, default_val);
    // ENABLE_SCREEN_PRINTF;
    cout << "[Ros_parameter]: " << parameter_name << " ==> " << parameter << std::endl;
    return parameter;
}

template <typename T>
T GetROSParameter(ros::NodeHandle &nh, const std::string parameter_name, T &parameter, T default_val)
{
    nh.param<T>(parameter_name.c_str(), parameter, default_val);
    // ENABLE_SCREEN_PRINTF;
    cout << "[Ros_parameter]: " << parameter_name << " ==> " << parameter << std::endl;
    return parameter;
}

template <typename T = double>
inline Eigen::Matrix<T, 3, 3> vec_to_hat(Eigen::Matrix<T, 3, 1> &omega)
{
    Eigen::Matrix<T, 3, 3> res_mat_33;
    res_mat_33.setZero();
    res_mat_33(0, 1) = -omega(2);
    res_mat_33(1, 0) = omega(2);
    res_mat_33(0, 2) = omega(1);
    res_mat_33(2, 0) = -omega(1);
    res_mat_33(1, 2) = -omega(0);
    res_mat_33(2, 1) = omega(0);
    return res_mat_33;
}

template <typename T = double>
T cot(const T theta)
{
    return 1.0 / std::tan(theta);
}

template <typename T = double>
Eigen::Matrix<T, 3, 3> inverse_right_jacobian_of_rotion_matrix(const Eigen::Matrix<T, 3, 1> &omega)
{
    // Barfoot, Timothy D, State estimation for robotics. Page 232-237
    Eigen::Matrix<T, 3, 3> res_mat_33;

    T theta = omega.norm();
    if (std::isnan(theta) || theta == 0)
        return Eigen::Matrix<T, 3, 3>::Identity();
    Eigen::Matrix<T, 3, 1> a = omega / theta;
    Eigen::Matrix<T, 3, 3> hat_a = vec_to_hat(a);
    res_mat_33 = (theta / 2) * (cot(theta / 2)) * Eigen::Matrix<T, 3, 3>::Identity() + (1 - (theta / 2) * (cot(theta / 2))) * a * a.transpose() + (theta / 2) * hat_a;
    // cout << "Omega: " << omega.transpose() << endl;
    // cout << "Res_mat_33:\r\n"  <<res_mat_33 << endl;
    return res_mat_33;
}

struct MeasureGroup // Lidar data and imu dates for the curent process
{
    MeasureGroup() { this->lidar.reset(new PointCloudXYZI()); };
    double lidar_beg_time;
    double lidar_sec_time;
    PointCloudXYZI::Ptr lidar;
    deque<sensor_msgs::Imu::ConstPtr> imu;
};

struct Camera_Lidar_queue
{
    double m_first_imu_time = -3e8;
    double m_sliding_window_tim = 10000;
    double m_last_imu_time = -3e8;
    double m_last_visual_time = -3e8;
    double m_visual_init_time = 3e88;
    double m_lidar_drag_cam_tim = 5.0;
    double m_if_lidar_start_first = 1;
    double m_camera_imu_td = 0;

    int m_if_acc_mul_G = 0;

    int m_if_have_lidar_data = 0;
    int m_if_have_camera_data = 0;
    int m_if_lidar_can_start = 1;
    Eigen::Vector3d g_noise_cov_acc;
    Eigen::Vector3d g_noise_cov_gyro;

    std::string m_bag_file_name;
    int m_if_write_res_to_bag = 0;
    int m_if_dump_log = 1;
    rosbag::Bag m_bag_for_record;

    // std::deque<PointCloudXYZI::Ptr> *m_liar_frame_buf = nullptr;
    std::deque<double> *m_lidar_time_buf = nullptr;

    void init_rosbag_for_recording()
    {
        if (m_if_write_res_to_bag)
        {
            cout << ANSI_COLOR_YELLOW_BG << "Record result to " << m_bag_file_name << ANSI_COLOR_RESET << endl;
            m_bag_for_record.open(m_bag_file_name.c_str(), rosbag::bagmode::Write);
        }
    }

    Camera_Lidar_queue()
    {
        m_if_have_lidar_data = 0;
        m_if_have_camera_data = 0;
    };
    ~Camera_Lidar_queue(){};

    void imu_in(const double &in_time)
    {
        if (m_first_imu_time < 0)
        {
            m_first_imu_time = in_time;
        }
        m_last_imu_time = std::max(in_time, m_last_imu_time);
        // m_last_imu_time = in_time;
    }

    int lidar_in(const double &in_time)
    {
        // cout << "LIDAR in " << endl;
        if (m_if_have_lidar_data == 0)
        {
            m_if_have_lidar_data = 1;
            cout << ANSI_COLOR_BLUE_BOLD << "Have LiDAR data" << endl;
        }
        if (in_time < m_last_imu_time - m_sliding_window_tim)
        {
            std::cout << ANSI_COLOR_RED_BOLD << "LiDAR incoming frame too old, need to be drop!!!" << ANSI_COLOR_RESET << std::endl;
            // TODO: Drop LiDAR frame
        }
        return 1;
    }

    int camera_in(const double &in_time)
    {
        if (in_time < m_last_imu_time - m_sliding_window_tim)
        {
            std::cout << ANSI_COLOR_RED_BOLD << "Camera incoming frame too old, need to be drop!!!" << ANSI_COLOR_RESET << std::endl;
            // TODO: Drop camera frame
        }
        return 1;
    }

    double get_lidar_front_time()
    {
        // if (m_liar_frame_buf != nullptr && m_liar_frame_buf->size())
        // {
        //     // return m_liar_frame_buf->front()->header.stamp.toSec() + 0.1;
        //     // return m_liar_frame_buf->front()->header.stamp;
        // }
        if (m_lidar_time_buf != nullptr && m_lidar_time_buf->size())
        {
            return m_lidar_time_buf->front() + 0.1;
        }
        else
        {
            return -3e8;
        }
    }

    double get_camera_front_time()
    {
        return m_last_visual_time + m_camera_imu_td;
        // if (m_camera_frame_buf != nullptr && m_camera_frame_buf->size())
        // {
        //     double min_time;

        //     // return std::min(m_camera_frame_buf->front()->second()->header.stamp.toSec());
        //     return (m_camera_frame_buf->front().second->header.stamp.toSec());
        // }
        // else
        // {
        //     return -3e8;
        // }
    }

    bool if_camera_can_process()
    {
        m_if_have_camera_data = 1;
        double cam_last_time = get_camera_front_time();
        double lidar_last_time = get_lidar_front_time();
        if (m_if_have_lidar_data != 1)
        {
            // scope_color(ANSI_COLOR_YELLOW_BOLD);
            // cout << "Camera can update , no LiDAR data" << endl;
            // printf_line;
            return true;
        }

        if (cam_last_time < 0 || lidar_last_time < 0)
        {
            // printf_line;
            return false;
        }

        // if (get_camera_front_time() > m_last_imu_time - m_sliding_window_tim)
        // {
        //     return false;
        // }

        if (lidar_last_time <= cam_last_time)
        {
            // LiDAR data need process first.
            return false;
        }
        else
        {
            // scope_color(ANSI_COLOR_YELLOW_BOLD);
            // cout << "Camera can update, " << get_lidar_front_time() - m_first_imu_time << " | " << get_camera_front_time() - m_first_imu_time << endl;
            return true;
        }
        return false;
    }

    bool if_lidar_can_process()
    {
        // m_if_have_lidar_data = 1;
        double cam_last_time = get_camera_front_time();
        double lidar_last_time = get_lidar_front_time();

        if (m_if_have_camera_data == 0)
        {
            // scope_color(ANSI_COLOR_BLUE_BOLD);
            // cout << "LiDAR can update , no camera data" << endl;
            // printf_line;
            return true;
        }

        if (cam_last_time < 0 || lidar_last_time < 0)
        {
            // printf_line;
            // cout << "Cam_tim = " << cam_last_time << ", lidar_last_time = " << lidar_last_time << endl;
            return false;
        }

        // if (get_lidar_front_time() > m_last_imu_time - m_sliding_window_tim)
        // {
        //     return false;
        // }

        if (lidar_last_time > cam_last_time)
        {
            // Camera data need process first.
            ;
            return false;
        }
        else
        {
            // scope_color(ANSI_COLOR_BLUE_BOLD);
            // cout << "LiDAR can update, " << get_lidar_front_time() - m_first_imu_time << " | " << get_camera_front_time() - m_first_imu_time << endl;
            // printf_line;
            return true;
        }
        return false;
    }
};

struct StatesGroup
{
    StatesGroup()
    {
        this->rot_end = Eigen::Matrix3d::Identity();
        this->pos_end = Zero3d;
        this->vel_end = Zero3d;
        this->bias_g = Zero3d;
        this->bias_a = Zero3d;
        this->gravity = Eigen::Vector3d(0.0, 0.0, 9.805);
        this->cov = Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES>::Identity() * INIT_COV;
        this->last_update_time = 0;
    };
    ~StatesGroup()
    {
    }

    StatesGroup(const StatesGroup &b)
    {
        this->rot_end = b.rot_end;
        this->pos_end = b.pos_end;
        this->vel_end = b.vel_end;
        this->bias_g = b.bias_g;
        this->bias_a = b.bias_a;
        this->gravity = b.gravity;
        this->cov = b.cov;
        this->last_update_time = b.last_update_time;
    };

    StatesGroup &operator=(const StatesGroup &b)
    {
        this->rot_end = b.rot_end;
        this->pos_end = b.pos_end;
        this->vel_end = b.vel_end;
        this->bias_g = b.bias_g;
        this->bias_a = b.bias_a;
#if ESTIMATE_GRAVITY
        this->gravity = b.gravity;
#else
        this->gravity = Eigen::Vector3d(0.0, 0.0, 9.805);
#endif
        this->cov = b.cov;
        this->last_update_time = b.last_update_time;
        return *this;
    };

    StatesGroup operator+(const Eigen::Matrix<double, DIM_OF_STATES, 1> &state_add)
    {
        StatesGroup a;
        a.rot_end = this->rot_end * Exp(state_add(0, 0), state_add(1, 0), state_add(2, 0));
        a.pos_end = this->pos_end + state_add.block<3, 1>(3, 0);
        a.vel_end = this->vel_end + state_add.block<3, 1>(6, 0);
        a.bias_g = this->bias_g + state_add.block<3, 1>(9, 0);
        a.bias_a = this->bias_a + state_add.block<3, 1>(12, 0);
#if ESTIMATE_GRAVITY
        a.gravity = this->gravity + state_add.block<3, 1>(15, 0);
#endif

        a.cov = this->cov;
        a.last_update_time = this->last_update_time;
        return a;
    };

    StatesGroup &operator+=(const Eigen::Matrix<double, DIM_OF_STATES, 1> &state_add)
    {
        this->rot_end = this->rot_end * Exp(state_add(0, 0), state_add(1, 0), state_add(2, 0));
        this->pos_end += state_add.block<3, 1>(3, 0);
        this->vel_end += state_add.block<3, 1>(6, 0);
        this->bias_g += state_add.block<3, 1>(9, 0);
        this->bias_a += state_add.block<3, 1>(12, 0);
#if ESTIMATE_GRAVITY
        this->gravity += state_add.block<3, 1>(15, 0);
#endif
        return *this;
    };

    Eigen::Matrix<double, DIM_OF_STATES, 1> operator-(const StatesGroup &b)
    {
        Eigen::Matrix<double, DIM_OF_STATES, 1> a;
        Eigen::Matrix3d rotd(b.rot_end.transpose() * this->rot_end);
        a.block<3, 1>(0, 0) = Log(rotd);
        a.block<3, 1>(3, 0) = this->pos_end - b.pos_end;
        a.block<3, 1>(6, 0) = this->vel_end - b.vel_end;
        a.block<3, 1>(9, 0) = this->bias_g - b.bias_g;
        a.block<3, 1>(12, 0) = this->bias_a - b.bias_a;
        a.block<3, 1>(15, 0) = this->gravity - b.gravity;
        return a;
    };

    static void display(const StatesGroup &state, std::string str = std::string("State: "))
    {
        V3D angle_axis = Log(state.rot_end) * 57.3;
        printf("%s |", str.c_str());
        printf("[%.5f] | ", state.last_update_time);
        printf("(%.3f, %.3f, %.3f) | ", angle_axis(0), angle_axis(1), angle_axis(2));
        printf("(%.3f, %.3f, %.3f) | ", state.pos_end(0), state.pos_end(1), state.pos_end(2));
        printf("(%.3f, %.3f, %.3f) | ", state.vel_end(0), state.vel_end(1), state.vel_end(2));
        printf("(%.3f, %.3f, %.3f) | ", state.bias_g(0), state.bias_g(1), state.bias_g(2));
        printf("(%.3f, %.3f, %.3f) \r\n", state.bias_a(0), state.bias_a(1), state.bias_a(2));
    }

    Eigen::Matrix3d rot_end;                                 // the estimated attitude (rotation matrix) at the end lidar point
    Eigen::Vector3d pos_end;                                 // the estimated position at the end lidar point (world frame)
    Eigen::Vector3d vel_end;                                 // the estimated velocity at the end lidar point (world frame)
    Eigen::Vector3d bias_g;                                  // gyroscope bias
    Eigen::Vector3d bias_a;                                  // accelerator bias
    Eigen::Vector3d gravity;                                 // the estimated gravity acceleration
    Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES> cov; // states covariance
    double last_update_time = 0;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template <typename T>
T rad2deg(T radians) { return radians * 180.0 / PI_M; }

template <typename T>
T deg2rad(T degrees) { return degrees * PI_M / 180.0; }

template <typename T>
auto set_pose6d(const double t, const Matrix<T, 3, 1> &a,
                const Matrix<T, 3, 1> &g, const Matrix<T, 3, 1> &v,
                const Matrix<T, 3, 1> &p, const Matrix<T, 3, 3> &R)
{
    Pose6D rot_kp;
    rot_kp.offset_time = t;
    for (int i = 0; i < 3; i++)
    {
        rot_kp.acc[i] = a(i);
        rot_kp.gyr[i] = g(i);
        rot_kp.vel[i] = v(i);
        rot_kp.pos[i] = p(i);
        for (int j = 0; j < 3; j++)
            rot_kp.rot[i * 3 + j] = R(i, j);
    }
    // Map<M3D>(rot_kp.rot, 3,3) = R;
    return move(rot_kp);
}

/* comment
plane equation: Ax + By + Cz + D = 0
convert to: A/D*x + B/D*y + C/D*z = -1
solve: A0*x0 = b0
where A0_i = [x_i, y_i, z_i], x0 = [A/D, B/D, C/D]^T, b0 = [-1, ..., -1]^T
normvec:  normalized x0
*/
template <typename T>
bool esti_normvector(Matrix<T, 3, 1> &normvec, const PointVector &point,
                     const T &threshold, const int &point_num)
{
    MatrixXf A(point_num, 3);
    MatrixXf b(point_num, 1);
    b.setOnes();
    b *= -1.0f;

    for (int j = 0; j < point_num; j++)
    {
        A(j, 0) = point[j].x;
        A(j, 1) = point[j].y;
        A(j, 2) = point[j].z;
    }
    normvec = A.colPivHouseholderQr().solve(b);

    for (int j = 0; j < point_num; j++)
    {
        if (fabs(normvec(0) * point[j].x + normvec(1) * point[j].y +
                 normvec(2) * point[j].z + 1.0f) > threshold)
        {
            return false;
        }
    }

    normvec.normalize();
    return true;
}

template <typename T>
bool esti_plane(Matrix<T, 4, 1> &pca_result, const PointVector &point,
                const T &threshold)
{
    Matrix<T, NUM_MATCH_POINTS, 3> A;
    Matrix<T, NUM_MATCH_POINTS, 1> b;
    b.setOnes();
    b *= -1.0f;

    for (int j = 0; j < NUM_MATCH_POINTS; j++)
    {
        A(j, 0) = point[j].x;
        A(j, 1) = point[j].y;
        A(j, 2) = point[j].z;
    }

    Matrix<T, 3, 1> normvec = A.colPivHouseholderQr().solve(b);

    for (int j = 0; j < NUM_MATCH_POINTS; j++)
    {
        if (fabs(normvec(0) * point[j].x + normvec(1) * point[j].y +
                 normvec(2) * point[j].z + 1.0f) > threshold)
        {
            return false;
        }
    }

    T n = normvec.norm();
    pca_result(0) = normvec(0) / n;
    pca_result(1) = normvec(1) / n;
    pca_result(2) = normvec(2) / n;
    pca_result(3) = 1.0 / n;
    return true;
}

#endif