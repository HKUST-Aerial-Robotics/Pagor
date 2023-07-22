//
// Created by qzj on 23-2-22.
//
#include "utility.h"

//Velodyne 64 HDE
// const int N_SCAN = 64;
// const int Horizon_SCAN = 1800; //1028~4500
// const float ang_res_x = 360.0/float(Horizon_SCAN);
// const float ang_res_y = 26.9/float(N_SCAN-1);//28.0/float(N_SCAN-1);
// const float ang_bottom = 25.0;
// const int groundScanInd = 60;    // 60 ;

// VLP-16
// const int N_SCAN = 16;
// const int Horizon_SCAN = 1800;
// const float ang_res_x = 0.2;
// const float ang_res_y = 2.0;
// const float ang_bottom = 15.0+0.1;
// const int groundScanInd = 7;

// HDL-32E
// const int N_SCAN = 32;
// const int Horizon_SCAN = 1800;
// const float ang_res_x = 360.0/float(Horizon_SCAN);
// const float ang_res_y = 41.33/float(N_SCAN-1);
// const float ang_bottom = 30.67;
// const int groundScanInd = 20;

// Ouster users may need to uncomment line 159 in imageProjection.cpp
// Usage of Ouster imu data is not fully supported yet, please just publish point cloud data
// Ouster OS1-16
// const int N_SCAN = 16;
// const int Horizon_SCAN = 1024;
// const float ang_res_x = 360.0/float(Horizon_SCAN);
// const float ang_res_y = 33.2/float(N_SCAN-1);
// const float ang_bottom = 16.6+0.1;
// const int groundScanInd = 7;

// Ouster OS1-64
//  const int N_SCAN = 64;
//  const int Horizon_SCAN = 1024;
//  const float ang_res_x = 360.0/float(Horizon_SCAN);
//  const float ang_res_y = 33.2/float(N_SCAN-1);
//  const float ang_bottom = 16.6+0.1;
//  const int groundScanInd = 15;

const bool loopClosureEnableFlag = true;
const double mappingProcessInterval = 0.3;

const float scanPeriod = 0.1;
const int systemDelay = 0;
const int imuQueLength = 200;

// const float sensorMountAngle = 0.0;

// //segmentation threshold
// const float segmentTheta = 60.0/180.0*M_PI; // decrese this value may improve accuracy       //60.0/180.0*M_PI

// //If number of segment is below than 30, check line number. this for minimum number of point for it
// const int segmentValidPointNum = 5;

// //if number of segment is small, number of line is checked, this is threshold for it.
// const int segmentValidLineNum = 3;

// const float segmentAlphaX = ang_res_x / 180.0 * M_PI;
// const float segmentAlphaY = ang_res_y / 180.0 * M_PI;


//const int edgeFeatureNum = 2;
//const int surfFeatureNum = 4;
//const int sectionsTotal = 6;
//const float edgeThreshold = 0.1;
//const float surfThreshold = 0.1;
//const float nearestFeatureSearchSqDist = 25;
//
//
//// Mapping Params
//const float surroundingKeyframeSearchRadius = 50.0; // key frame that is within n meters from current pose will be considerd for scan-to-map optimization (when loop closure disabled)
//const int   surroundingKeyframeSearchNum = 50; // submap size (when loop closure enabled)
//// history key frames (history submap for loop closure)
//const float historyKeyframeSearchRadius = 7.0; // key frame that is within n meters from current pose will be considerd for loop closure
//const int   historyKeyframeSearchNum = 25; // 2n+1 number of hostory key frames will be fused into a submap for loop closure
//const float historyKeyframeFitnessScore = 0.3; // the smaller the better alignment
//
//const float globalMapVisualizationSearchRadius = 500.0; // key frames with in n meters will be visualized

void
setCorrespondenceMarker(const pcl::PointCloud <PointType> &src_matched, const pcl::PointCloud <PointType> &tgt_matched,
                        visualization_msgs::Marker &marker, float thickness, std::vector<float> rgb_color, int id,
                        bool lift_src_z) {
    if (!marker.points.empty()) marker.points.clear();
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = id; // To avoid overlap
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x = thickness; // thickness
    marker.color.r = rgb_color[0];
    marker.color.g = rgb_color[1];
    marker.color.b = rgb_color[2];
    marker.color.a = 1.0; // Don't forget to set the alpha!

    geometry_msgs::Point srcP;
    geometry_msgs::Point tgtP;
    assert(src_matched.size() == tgt_matched.size());
    for (int idx = 0; idx < src_matched.size(); ++idx) {
        PointType sP = src_matched[idx];
        PointType sT = tgt_matched[idx];
        srcP.x = sP.x;
        srcP.y = sP.y;
        srcP.z = sP.z;
        tgtP.x = sT.x;
        tgtP.y = sT.y;
        tgtP.z = sT.z;
        if (lift_src_z) srcP.z += 10;
        marker.points.emplace_back(srcP);
        marker.points.emplace_back(tgtP);
    }
}
