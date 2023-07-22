//
// Created on 19-3-27.
//

#ifndef PROJECT_COMMONFUNC_H
#define PROJECT_COMMONFUNC_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <tf/transform_broadcaster.h>
#include <vector>
#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

inline std::string zfill(int num, int width) {
    std::stringstream ss;
    ss << std::setw(width) << std::setfill('0') << num;
    return ss.str();
}

template<typename T>
inline T dcm2angle(const Eigen::Matrix<T, 3, 3> &R, bool is_degree = false) {
    double a = (R(0, 0) + R(1, 1) + R(2, 2) - 1) / 2;
    if (a > 1) {
        a = 1;
    } else if (a < -1) {
        a = -1;
    }
    T angle = acos(a);
    if (is_degree) {
        angle = angle * 180.0 / M_PI;
    }
    return angle;
}

template<typename T>
inline Eigen::Matrix<T, 3, 3> skew(const Eigen::Matrix<T, 3, 1> &axis) {
    Eigen::Matrix<T, 3, 3> skew_matrix = Eigen::Matrix<T, 3, 3>::Identity();

    skew_matrix << 0, -axis(2, 0), axis(1, 0),
            axis(2, 0), 0, -axis(0, 0),
            -axis(1, 0), axis(0, 0), 0;

    return skew_matrix;
}

template<typename T>
inline Eigen::Matrix<T, 3, 1> dcm2rpy(const Eigen::Matrix<T, 3, 3> &R) {
    Eigen::Matrix<T, 3, 1> rpy;
    rpy[1] = atan2(-R(2, 0), sqrt(pow(R(0, 0), 2) + pow(R(1, 0), 2)));
    if (fabs(rpy[1] - M_PI / 2) < 0.00001) {
        rpy[2] = 0;
        rpy[0] = -atan2(R(0, 1), R(1, 1));
    } else {
        if (fabs(rpy[1] + M_PI / 2) < 0.00001) {
            rpy[2] = 0;
            rpy[0] = -atan2(R(0, 1), R(1, 1));
        } else {
            rpy[2] = atan2(R(1, 0) / cos(rpy[1]), R(0, 0) / cos(rpy[1]));
            rpy[0] = atan2(R(2, 1) / cos(rpy[1]), R(2, 2) / cos(rpy[1]));
        }
    }
    return rpy;
}

template<typename T>
void toEulerAngle(const Eigen::Quaternion<T> &q, T &roll, T &pitch, T &yaw) {
    // roll (x-axis rotation)
    T sinr_cosp = 2.0 * (q.w() * q.x() + q.y() * q.z());
    T cosr_cosp = 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
    roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    T sinp = 2.0 * (q.w() * q.y() - q.z() * q.x());
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = asin(sinp);

    // yaw (z-axis rotation)
    T siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
    T cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    yaw = atan2(siny_cosp, cosy_cosp);
}


#endif //PROJECT_COMMONFUNC_H
