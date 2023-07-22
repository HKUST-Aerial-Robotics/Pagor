#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_


#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>
#include <experimental/filesystem>

using namespace std;
using PointType = pcl::PointXYZ;

// added to adapt to colored point clouds
using PointRGB = pcl::PointXYZRGB;
using PointL = pcl::PointXYZL;

typedef pcl::PointXYZI PointTypeIP;

enum class RegularizationMethod {
    NONE, MIN_EIG, NORMALIZED_MIN_EIG, PLANE, FROBENIUS
};

//Velodyne 64 HDE
// extern const int N_SCAN = 64;
// extern const int Horizon_SCAN = 1800; //1028~4500
// extern const float ang_res_x = 360.0/float(Horizon_SCAN);
// extern const float ang_res_y = 26.9/float(N_SCAN-1);//28.0/float(N_SCAN-1);
// extern const float ang_bottom = 25.0;
// extern const int groundScanInd = 60;    // 60 ;

// VLP-16
// extern const int N_SCAN = 16;
// extern const int Horizon_SCAN = 1800;
// extern const float ang_res_x = 0.2;
// extern const float ang_res_y = 2.0;
// extern const float ang_bottom = 15.0+0.1;
// extern const int groundScanInd = 7;

// HDL-32E
// extern const int N_SCAN = 32;
// extern const int Horizon_SCAN = 1800;
// extern const float ang_res_x = 360.0/float(Horizon_SCAN);
// extern const float ang_res_y = 41.33/float(N_SCAN-1);
// extern const float ang_bottom = 30.67;
// extern const int groundScanInd = 20;

// Ouster users may need to uncomment line 159 in imageProjection.cpp
// Usage of Ouster imu data is not fully supported yet, please just publish point cloud data
// Ouster OS1-16
// extern const int N_SCAN = 16;
// extern const int Horizon_SCAN = 1024;
// extern const float ang_res_x = 360.0/float(Horizon_SCAN);
// extern const float ang_res_y = 33.2/float(N_SCAN-1);
// extern const float ang_bottom = 16.6+0.1;
// extern const int groundScanInd = 7;

// Ouster OS1-64
//  extern const int N_SCAN = 64;
//  extern const int Horizon_SCAN = 1024;
//  extern const float ang_res_x = 360.0/float(Horizon_SCAN);
//  extern const float ang_res_y = 33.2/float(N_SCAN-1);
//  extern const float ang_bottom = 16.6+0.1;
//  extern const int groundScanInd = 15;

extern const bool loopClosureEnableFlag;
extern const double mappingProcessInterval;

extern const float scanPeriod;
extern const int systemDelay;
extern const int imuQueLength;

// extern const float sensorMountAngle = 0.0;

// //segmentation threshold
// extern const float segmentTheta = 60.0/180.0*M_PI; // decrese this value may improve accuracy       //60.0/180.0*M_PI

// //If number of segment is below than 30, check line number. this for minimum number of point for it
// extern const int segmentValidPointNum = 5;

// //if number of segment is small, number of line is checked, this is threshold for it.
// extern const int segmentValidLineNum = 3;

// extern const float segmentAlphaX = ang_res_x / 180.0 * M_PI;
// extern const float segmentAlphaY = ang_res_y / 180.0 * M_PI;


//extern const int edgeFeatureNum = 2;
//extern const int surfFeatureNum = 4;
//extern const int sectionsTotal = 6;
//extern const float edgeThreshold = 0.1;
//extern const float surfThreshold = 0.1;
//extern const float nearestFeatureSearchSqDist = 25;
//
//
//// Mapping Params
//extern const float surroundingKeyframeSearchRadius = 50.0; // key frame that is within n meters from current pose will be considerd for scan-to-map optimization (when loop closure disabled)
//extern const int   surroundingKeyframeSearchNum = 50; // submap size (when loop closure enabled)
//// history key frames (history submap for loop closure)
//extern const float historyKeyframeSearchRadius = 7.0; // key frame that is within n meters from current pose will be considerd for loop closure
//extern const int   historyKeyframeSearchNum = 25; // 2n+1 number of hostory key frames will be fused into a submap for loop closure
//extern const float historyKeyframeFitnessScore = 0.3; // the smaller the better alignment
//
//extern const float globalMapVisualizationSearchRadius = 500.0; // key frames with in n meters will be visualized


struct smoothness_t {
    float value;
    size_t ind;
};

struct by_value {
    bool operator()(smoothness_t const &left, smoothness_t const &right) {
        return left.value < right.value;
    }
};

/*
    * A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp)
    */
struct PointXYZIRPYT {
    PCL_ADD_POINT4D

    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x)(float, y, y)
                                           (float, z, z)(float, intensity, intensity)
                                           (float, roll, roll)(float, pitch, pitch)(float, yaw, yaw)
                                           (double, time, time)
)

typedef PointXYZIRPYT PointTypePose;

void
setCorrespondenceMarker(const pcl::PointCloud<PointType> &src_matched, const pcl::PointCloud<PointType> &tgt_matched,
                        visualization_msgs::Marker &marker, float thickness = 0.1,
                        std::vector<float> rgb_color = {0.0, 0.0, 0.0}, int id = 0,
                        bool lift_src_z = false);

template<typename T>
struct hash_eigen {
    std::size_t operator()(T const &matrix) const {
        size_t seed = 0;
        for (int i = 0; i < (int) matrix.size(); i++) {
            auto elem = *(matrix.data() + i);
            seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 +
                    (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};


#endif
