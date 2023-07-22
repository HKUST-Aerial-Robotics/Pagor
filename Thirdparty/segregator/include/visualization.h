//
// Created by qzj on 23-2-22.
//

#ifndef SRC_VISUALIZATION_H
#define SRC_VISUALIZATION_H

#include "filtering.hpp"
#include "cluster_manager.hpp"

pcl::PointCloud<PointType>::ConstPtr getCloud(std::string filename);

void setParams(int semantic_class, double cluster_distance_threshold, int minNum, int maxNum,
               clusterManager::ClusterParams &params, clusterManager::DCVCParam &seg_param);

void merge_label(const string label_file_path, pcl::PointCloud<PointType>::Ptr raw_pc,
                 pcl::PointCloud<PointL>::Ptr semantic_pc, double label_deter_rate);

void apply_color_mapping_spvnas(int label, int &r, int &g, int &b);

void color_pc(const pcl::PointCloud<PointL>::Ptr semantic_cloud, pcl::PointCloud<PointRGB>::Ptr colored_cloud,
              bool lift_z = false);

void setCovMatsMarkers(visualization_msgs::MarkerArray &markerArray, const pcl::PointCloud<PointType>::Ptr cloud,
                       const std::vector<Eigen::Matrix3d> &covariances, const std::vector<float> rgb_color, int id);

pcl::PointCloud<PointL>::Ptr random_downsample_pl(pcl::PointCloud<PointL>::Ptr cloud_ori, int ratio);

#endif //SRC_VISUALIZATION_H
