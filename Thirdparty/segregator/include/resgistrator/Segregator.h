//
// Created by qzj on 23-2-22.
//

#ifndef SRC_SEGREGATOR_H
#define SRC_SEGREGATOR_H

#include <ros/ros.h>
#include <ros/package.h>
#include <locale>
#include <yaml-cpp/yaml.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>
#include <eigen_conversions/eigen_msg.h>
#include <visualization_msgs/Marker.h>

#include "filtering.hpp"
#include "cluster_manager.hpp"
#include "semantic_teaser.hpp"
#include "dataio.hpp"
#include "glog/logging.h"
#include "clipper/clipper.h"


class Segregator {
public:
    YAML::Node config_node_;
    std::string package_path, config_path_;
    bool visualize = false;
    bool use_clipper = true;

    std::shared_ptr<semanticTeaser> semSolver_ptr;
    std::shared_ptr<clipper::CLIPPER> clipper_ptr;

    pcl::PointCloud<PointType>::Ptr srcRaw;
    pcl::PointCloud<PointType>::Ptr tgtRaw;
    pcl::PointCloud<PointType>::Ptr transformed_src;
    pcl::PointCloud<PointL>::Ptr srcSemanticPc;
    pcl::PointCloud<PointL>::Ptr tgtSemanticPc;

    std::vector<pcl::PointCloud<PointType>> src_sem_vec;
    std::vector<Eigen::Matrix3d> src_covariances;
    std::vector<pcl::PointCloud<PointType>> tgt_sem_vec;
    std::vector<Eigen::Matrix3d> tgt_covariances;

    // backgroud points feature extraction
    pcl::PointCloud<PointType>::Ptr src_matched_bg;
    std::vector<Eigen::Matrix3d> src_bg_covariances;
    pcl::PointCloud<PointType>::Ptr tgt_matched_bg;
    std::vector<Eigen::Matrix3d> tgt_bg_covariances;

    // building cluster params
    int building_class_num;
    double building_min_cluster_dist;
    int building_min_point_num, building_max_point_num;
    bool use_building;
    bool use_DCVC_building;
    int building_minSeg;

    // car cluster params
    int car_class_num;
    double car_min_cluster_dist;
    int car_min_point_num, car_max_point_num;
    bool use_car;
    bool use_DCVC_car;
    int car_minSeg;

    // vegetation cluster params
    int vegetation_class_num;
    double vegetation_min_cluster_dist;
    int vegetation_min_point_num, vegetation_max_point_num;
    bool use_veg;
    bool use_DCVC_veg;
    int veg_minSeg;

    // trunk cluster params
    int trunk_class_num;
    double trunk_min_cluster_dist;
    int trunk_min_point_num, trunk_max_point_num;
    bool use_trunk;
    bool use_DCVC_trunk;
    int trunk_minSeg;

    // DCVC segmentation params
    double startR, deltaR, deltaP, deltaA;
    int minSeg;

    // registration parmas
    double noise_level;
    double distribution_noise_level;
    int src_indx, tgt_indx;
    bool solving_w_cov;

    // comparative results
    double inital_yaw_rate;
    double label_deter_rate;

    clusterManager::DCVCParam building_DCVC_param;
    clusterManager::DCVCParam car_DCVC_param;
    clusterManager::DCVCParam veg_DCVC_param;
    clusterManager::DCVCParam trunk_DCVC_param;

    clusterManager::ClusterParams car_params;
    clusterManager::ClusterParams building_params;
    clusterManager::ClusterParams veg_params;
    clusterManager::ClusterParams trunk_params;

public:
    Segregator(std::string config_file, bool visualize = false);

    void LoadConfig(std::string config_file);

    void reset(double noise_level = -1.0, double distribution_noise_level = -1.0);

    void LoadPCDLabFromFiles(std::string src_path, std::string tgt_path,
                             std::string src_label_path, std::string tgt_label_path);

    void TransToGaussian(pcl::PointCloud<PointL>::Ptr semanticPc,
                         std::vector<pcl::PointCloud<PointType>> &sem_vec,
                         std::vector<Eigen::Matrix3d> &covariances);

    void
    TransToGaussianBg(pcl::PointCloud<PointType>::Ptr src_matched_bg, std::vector<Eigen::Matrix3d> &src_bg_covariances,
                      pcl::PointCloud<PointType>::Ptr tgt_matched_bg, std::vector<Eigen::Matrix3d> &tgt_bg_covariances);

    Eigen::Matrix4d GobalRegistration();

    void SaveKeyPoints();

    void GobalRegistration(clipper::CertifiedTransformations &solutions);

    double GetScore(const Eigen::Matrix4d &pose);

    double GetScore2(const Eigen::Matrix4d &pose);

    Eigen::Matrix4d SelectBestPose(const clipper::CertifiedTransformations &pose_candidates);

    void check(const Eigen::Matrix4d &pose);
};


#endif //SRC_SEGREGATOR_H
