/*****************************************************************
 *
 * Copyright (c) 2022, Nanyang Technological University, Singapore
 *
 * Authors: Pengyu Yin
 * Contact: pengyu001@e.ntu.edu.sg
 *
 * DCVC Code based on: T-LOAM: Truncated Least Squares LiDAR-Only Odometry and Mapping in Real Time
 * link: https://github.com/zpw6106/tloam
 * 
 * fpfh feature extraction/G-TRIM code based on: Teaser++ and Quatro
 * link: https://github.com/MIT-SPARK/TEASER-plusplus
 * link: https://github.com/url-kaist/Quatro
 * 
 ****************************************************************/

#ifndef CLUSTER_MANAGER_H
#define CLUSTER_MANAGER_H

#include <unordered_map>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "teaser_utils/fpfh.h"
#include "teaser_utils/feature_matcher.h"
#include "utility.h"

#include "conversion.hpp"
#include "glog/logging.h"

class clusterManager {

public:
    pcl::PointCloud<PointL>::Ptr sem_cloud_in;

    clusterManager() {
        tree_.reset(new pcl::search::KdTree<pcl::PointXYZ>);
        selected_points_.reset(new pcl::PointCloud<PointType>);
        selected_src_points_.reset(new pcl::PointCloud<PointType>);
        selected_tgt_points_.reset(new pcl::PointCloud<PointType>);
        src_descriptors_.reset(new pcl::PointCloud<pcl::FPFHSignature33>());
        tgt_descriptors_.reset(new pcl::PointCloud<pcl::FPFHSignature33>());
    }

    /** \brief Empty destructor */
    ~clusterManager() {
        tree_.reset(new pcl::search::KdTree<pcl::PointXYZ>);
        selected_points_.reset(new pcl::PointCloud<PointType>);
        selected_src_points_.reset(new pcl::PointCloud<PointType>);
        selected_tgt_points_.reset(new pcl::PointCloud<PointType>);
        src_descriptors_.reset(new pcl::PointCloud<pcl::FPFHSignature33>());
        tgt_descriptors_.reset(new pcl::PointCloud<pcl::FPFHSignature33>());
    }

    struct DCVCParam {
        double startR = 0.0;
        double deltaR = 0.0;
        double deltaP = 0.0;
        double deltaA = 0.0;
        int minSeg = 0;
    };

    struct ClusterParams {
        // semantic class
        int semanticLabel = 1;
        // DBScan clustering related params
        double clusterTolerance = 0.5;
        int minClusterSize = 20;
        int maxClusterSize = 2000;
        // DCVD params
        double startR = 0.0;
        double deltaR = 0.0;
        double deltaP = 0.0;
        double deltaA = 0.0;
        int minSeg = 0;
    };

    void reset(ClusterParams params) {
        params_ = params;
        reg_method_ = RegularizationMethod::NONE;
    }

    bool selectSemanticPoints(pcl::PointCloud<PointL>::Ptr input_sem_cloud) {
        sem_cloud_in = input_sem_cloud;

        for (int i = 0; i < sem_cloud_in->points.size(); i++) {
            if (sem_cloud_in->points[i].label == params_.semanticLabel) {
                PointType tmpPt;
                tmpPt.x = sem_cloud_in->points[i].x;
                tmpPt.y = sem_cloud_in->points[i].y;
                tmpPt.z = sem_cloud_in->points[i].z;
                selected_points_->points.push_back(tmpPt);
            }
        }
        if (selected_points_->points.size() < 0)
            return false;

        return true;
    }

    bool selectSemanticPoints(pcl::PointCloud<PointL>::Ptr input_src_sem_cloud,
                              pcl::PointCloud<PointL>::Ptr input_tgt_sem_cloud) {
        for (int i = 0; i < input_src_sem_cloud->points.size(); i++) {
            if (input_src_sem_cloud->points[i].label == params_.semanticLabel) {
                PointType tmpPt;
                tmpPt.x = input_src_sem_cloud->points[i].x;
                tmpPt.y = input_src_sem_cloud->points[i].y;
                tmpPt.z = input_src_sem_cloud->points[i].z;
                selected_src_points_->points.push_back(tmpPt);
            }
        }

        for (int i = 0; i < input_tgt_sem_cloud->points.size(); i++) {
            if (input_tgt_sem_cloud->points[i].label == params_.semanticLabel) {
                PointType tmpPt;
                tmpPt.x = input_tgt_sem_cloud->points[i].x;
                tmpPt.y = input_tgt_sem_cloud->points[i].y;
                tmpPt.z = input_tgt_sem_cloud->points[i].z;
                selected_tgt_points_->points.push_back(tmpPt);
            }
        }

        if (selected_src_points_->points.size() < 0 || selected_tgt_points_->points.size() < 0)
            return false;

        return true;
    }

    void dbscanSeg(pcl::PointCloud<PointL>::Ptr input_sem_cloud) {
        if (!selectSemanticPoints(input_sem_cloud)) {
            ROS_ERROR("Select semantic points failed!");
            return;
        }

        const int region_max_ = 14;
        int regions_[100];

        regions_[0] = 4;
        regions_[1] = 5;
        regions_[2] = 4;
        regions_[3] = 5;
        regions_[4] = 4;
        regions_[5] = 5;
        regions_[6] = 5;
        regions_[7] = 4;
        regions_[8] = 5;
        regions_[9] = 4;
        regions_[10] = 5;
        regions_[11] = 5;
        regions_[12] = 4;
        regions_[13] = 20;

        boost::array<std::vector<int>, region_max_> indices_array;

        /*** Divide the point cloud into nested circular regions ***/
        for (int i = 0; i < selected_points_->points.size(); i++) {
            float range = 0.0;
            for (int j = 0; j < region_max_; j++) {
                float d2 = selected_points_->points[i].x * selected_points_->points[i].x +
                           selected_points_->points[i].y * selected_points_->points[i].y +
                           selected_points_->points[i].z * selected_points_->points[i].z;
                if (d2 > range * range && d2 <= (range + regions_[j]) * (range + regions_[j])) {
                    indices_array[j].push_back(i);
                    break;
                }
                range += regions_[j];
            }
        }

        /*** Euclidean clustering ***/
        float tolerance = 0;
        int last_clusters_begin = 0;
        int last_clusters_end = 0;
        float k_merging_threshold_ = 3;
        float z_merging_threshold_ = 0;

        for (int i = 0; i < region_max_; i++) {
            tolerance += 0.1;
            if (indices_array[i].size() > params_.minClusterSize) {
                boost::shared_ptr<std::vector<int> > indices_array_ptr(new std::vector<int>(indices_array[i]));
                pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
                tree->setInputCloud(selected_points_, indices_array_ptr);

                std::vector<pcl::PointIndices> cluster_indices;
                pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
                ec.setClusterTolerance(tolerance);
                ec.setMinClusterSize(params_.minClusterSize);
                ec.setMaxClusterSize(params_.maxClusterSize);
                ec.setSearchMethod(tree);
                ec.setInputCloud(selected_points_);
                ec.setIndices(indices_array_ptr);
                ec.extract(cluster_indices);

                for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
                     it != cluster_indices.end(); it++) {
                    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
                    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
                        cluster->points.push_back(selected_points_->points[*pit]);
                    }

                    /*** Merge clusters_ separated by nested regions ***/
                    for (int j = last_clusters_begin; j < last_clusters_end; j++) {
                        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
                        int K = 1; //the number of neighbors to search for
                        std::vector<int> k_indices(K);
                        std::vector<float> k_sqr_distances(K);
                        kdtree.setInputCloud(cluster);
                        if (clusters_[j]->points.size() >= 1) {
                            if (kdtree.nearestKSearch(*clusters_[j], clusters_[j]->points.size() - 1, K, k_indices,
                                                      k_sqr_distances) > 0) {
                                if (k_sqr_distances[0] < k_merging_threshold_) {
                                    *cluster += *clusters_[j];
                                    clusters_.erase(clusters_.begin() + j);
                                    last_clusters_end--;
                                    //std::cerr << "k-merging: clusters_ " << j << " is merged" << std::endl; 
                                }
                            }
                        }
                    }
                    /**************************************************/

                    cluster->width = cluster->size();
                    cluster->height = 1;
                    cluster->is_dense = true;
                    clusters_.push_back(cluster);
                }

                /*** Merge z-axis clusters_ ***/
                // overlapping on BEV
                for (int j = last_clusters_end; j < clusters_.size(); j++) {
                    Eigen::Vector4f j_min, j_max;
                    pcl::getMinMax3D(*clusters_[j], j_min, j_max);
                    for (int k = j + 1; k < clusters_.size(); k++) {
                        Eigen::Vector4f k_min, k_max;
                        pcl::getMinMax3D(*clusters_[k], k_min, k_max);
                        if (std::max(std::min((double) j_max[0], (double) k_max[0]) -
                                     std::max((double) j_min[0], (double) k_min[0]), 0.0)
                            * std::max(std::min((double) j_max[1], (double) k_max[1]) -
                                       std::max((double) j_min[1], (double) k_min[1]), 0.0) > z_merging_threshold_) {
                            *clusters_[j] += *clusters_[k];
                            clusters_.erase(clusters_.begin() + k);
                            //std::cerr << "z-merging: clusters_ " << k << " is merged into " << j << std::endl; 
                        }
                    }
                }
                /*****************************/
                last_clusters_begin = last_clusters_end;
                last_clusters_end = clusters_.size();
            }
        }
    }

    void computeCentroidAndCov(pcl::PointCloud<PointType>::Ptr cluster_centroid,
                               std::vector<Eigen::Matrix3d> &covariances) {
        int cluster_size = clusters_.size();
        covariances.resize(cluster_size);

        for (int i = 0; i < cluster_size; ++i) {

            Eigen::Matrix<double, 3, -1> neighbors(3, clusters_[i]->size());

            for (int j = 0; j < clusters_[i]->points.size(); j++) {
                neighbors.col(j) = clusters_[i]->at(j).getVector3fMap().cast<double>();
            }

            // compute centroid coordinate
            Eigen::Matrix<double, 3, 1> center = neighbors.rowwise().mean();
            PointType p_temp((float) center(0), (float) center(1), (float) center(2));
            cluster_centroid->push_back(p_temp);
            // substract mean
            neighbors.colwise() -= center;
            // compute cov 3*3
            Eigen::Matrix3d cov = neighbors * neighbors.transpose() / clusters_[i]->size();

            if (reg_method_ == RegularizationMethod::NONE) {
                covariances[i] = cov;
            } else {
                std::cerr << "Error: RegularizationMethod not implemented!";
            }
        }
    }

    void computeCov(pcl::PointCloud<PointType>::Ptr original_pt_cloud,
                    std::vector<int> &index,
                    std::vector<Eigen::Matrix3d> &covariances) {
        int sample_size = index.size();
        Eigen::Matrix<double, 3, -1> neighbors(3, sample_size);
        for (size_t i = 0; i < index.size(); ++i) {
            int cur_idx = index[i];
            neighbors.col(i) = original_pt_cloud->at(cur_idx).getVector3fMap().cast<double>();
        }

        // compute centroid coordinate
        Eigen::Matrix<double, 3, 1> center = neighbors.rowwise().mean();
        // substract mean
        neighbors.colwise() -= center;
        // compute cov 3*3
        Eigen::Matrix3d cov = neighbors * neighbors.transpose() / sample_size;

        covariances.emplace_back(cov);
    }

    // following code for dynamic voxel segmentation
    bool segmentPointCloud(pcl::PointCloud<PointL>::Ptr input_sem_cloud) {
        // pointCloudL -> pointCloud
        if (!selectSemanticPoints(input_sem_cloud)) {
            ROS_ERROR("Select semantic points failed!");
            return false;
        } else {
            // step1, scan->polar coordinate
            convert2polar();
        }

        // step 2, create hash table
        createHashTable();

        // step 3, DCVC segmentation
        std::vector<int> labelInfo{};
        if (!DCVC(labelInfo)) {
            //ROS_ERROR("DCVC algorithm segmentation failure");
            return false;
        }

        // step 4, store segmentation results to clusters_
        labelAnalysis(labelInfo);

        return true;
    }

    void convert2polar() {
        if (selected_points_->points.size() == 0) {
            //ROS_ERROR("Point cloud empty in converting cartesian to polar!");
        }

        // culculate yaw angle(rad)
        auto azimuthCal = [&](double x, double y) -> double {
            auto angle = static_cast<double>(std::atan2(y, x));
            return angle > 0.0 ? angle * 180 / M_PI : (angle + 2 * M_PI) * 180 / M_PI;
        };

        size_t totalSize = selected_points_->points.size();
        polarCor.resize(totalSize);

        Eigen::Vector3d cur = Eigen::Vector3d::Zero();
        for (size_t i = 0; i < totalSize; ++i) {
            // polar pitch azimuth
            Eigen::Vector3d rpa = Eigen::Vector3d::Zero();
            cur(0) = selected_points_->points[i].x;
            cur(1) = selected_points_->points[i].y;
            cur(2) = selected_points_->points[i].z;
            rpa.x() = cur.norm();
            rpa.y() = std::asin(cur.z() / rpa.x()) * 180.0 / M_PI;
            rpa.z() = azimuthCal(cur.x(), cur.y());

            if (rpa.x() >= 120.0 || rpa.x() <= 0.5)
                continue;

            minPitch = rpa.y() < minPitch ? rpa.y() : minPitch;
            maxPitch = rpa.y() > maxPitch ? rpa.y() : maxPitch;
            minPolar = rpa.x() < minPolar ? rpa.x() : minPolar;
            maxPolar = rpa.x() > maxPolar ? rpa.x() : maxPolar;

            polarCor[i] = rpa;
        }

        polarCor.shrink_to_fit();

        polarNum = 0;
        polarBounds.clear();
        width = static_cast<int>(std::round(360.0 / params_.deltaA) + 1);
        height = static_cast<int>((maxPitch - minPitch) / params_.deltaP);
        double range = minPolar;
        int step = 1;
        while (range <= maxPolar) {
            range += (params_.startR - step * params_.deltaR);
            polarBounds.emplace_back(range);
            polarNum++, step++;
        }

    }

    void createHashTable() {
        size_t totalSize = polarCor.size();

        Eigen::Vector3d cur = Eigen::Vector3d::Zero();
        int polarIndex, pitchIndex, azimuthIndex, voxelIndex;
        voxelMap.reserve(totalSize);

        for (size_t item = 0; item < totalSize; ++item) {
            cur = polarCor[item];
            polarIndex = getPolarIndex(cur.x());
            pitchIndex = static_cast<int>(std::round((cur.y() - minPitch) / params_.deltaP));
            azimuthIndex = static_cast<int>(std::round(cur.z() / params_.deltaA));

            voxelIndex = (azimuthIndex * (polarNum + 1) + polarIndex) + pitchIndex * (polarNum + 1) * (width + 1);

            auto iter = voxelMap.find(voxelIndex);
            if (iter != voxelMap.end()) {
                //iter->second.index.emplace_back(item);
                iter->second.emplace_back(item);
            } else {
                std::vector<int> index{};
                index.emplace_back(item);
                voxelMap.insert(std::make_pair(voxelIndex, index));
            }
        }
    }

    /**
     * @brief get the index value in the polar radial direction
     * @param radius, polar diameter
     * @return polar diameter index
     */
    int getPolarIndex(double &radius) {

        for (auto r = 0; r < polarNum; ++r) {
            if (radius < polarBounds[r])
                return r;
        }
        return polarNum - 1;
    }

    /**
     * @brief the Dynamic Curved-Voxle Clustering algoithm for fast and precise point cloud segmentaiton
     * @param label_info, output the category information of each point
     * @return true if success otherwise false
     */
    bool DCVC(std::vector<int> &label_info) {

        int labelCount = 0;
        size_t totalSize = polarCor.size();
        if (totalSize <= 0) {
            //ROS_ERROR("points in the cloud not enough to complete the DCVC algorithm");
            return false;
        }

        label_info.resize(totalSize, -1);
        Eigen::Vector3d cur = Eigen::Vector3d::Zero();
        int polar_index, pitch_index, azimuth_index, voxel_index, currInfo, neighInfo;

        for (size_t i = 0; i < totalSize; ++i) {
            if (label_info[i] != -1)
                continue;
            cur = polarCor[i];

            polar_index = getPolarIndex(cur.x());
            pitch_index = static_cast<int>(std::round((cur.y() - minPitch) / params_.deltaP));
            azimuth_index = static_cast<int>(std::round(cur.z() / params_.deltaA));
            voxel_index = (azimuth_index * (polarNum + 1) + polar_index) + pitch_index * (polarNum + 1) * (width + 1);

            auto iter_find = voxelMap.find(voxel_index);
            std::vector<int> neighbors;
            if (iter_find != voxelMap.end()) {

                std::vector<int> KNN{};
                searchKNN(polar_index, pitch_index, azimuth_index, KNN);

                for (auto &k: KNN) {
                    iter_find = voxelMap.find(k);

                    if (iter_find != voxelMap.end()) {
                        neighbors.reserve(iter_find->second.size());
                        for (auto &id: iter_find->second) {
                            neighbors.emplace_back(id);
                        }
                    }
                }
            }

            neighbors.swap(neighbors);

            if (!neighbors.empty()) {
                for (auto &id: neighbors) {
                    currInfo = label_info[i];       // current label index
                    neighInfo = label_info[id];     // voxel label index
                    if (currInfo != -1 && neighInfo != -1 && currInfo != neighInfo) {
                        for (auto &seg: label_info) {
                            if (seg == currInfo)
                                seg = neighInfo;
                        }
                    } else if (neighInfo != -1) {
                        label_info[i] = neighInfo;
                    } else if (currInfo != -1) {
                        label_info[id] = currInfo;
                    } else {
                        continue;
                    }
                }
            }

            // If there is no category information yet, then create a new label information
            if (label_info[i] == -1) {
                labelCount++;
                label_info[i] = labelCount;
                for (auto &id: neighbors) {
                    label_info[id] = labelCount;
                }
            }
        }

        // free memory
        std::vector<Eigen::Vector3d>().swap(polarCor);

        return true;
    }

    /**
     * @brief search for neighboring voxels
     * @param polar_index, polar diameter index
     * @param pitch_index, pitch angular index
     * @param azimuth_index, azimuth angular index
     * @param out_neighIndex, output adjacent voxel index set
     * @return void
     */
    void searchKNN(int &polar_index, int &pitch_index, int &azimuth_index, std::vector<int> &out_neighIndex) const {

        for (auto z = pitch_index - 1; z <= pitch_index + 1; ++z) {
            if (z < 0 || z > height)
                continue;
            for (int y = polar_index - 1; y <= polar_index + 1; ++y) {
                if (y < 0 || y > polarNum)
                    continue;

                for (int x = azimuth_index - 1; x <= azimuth_index + 1; ++x) {
                    int ax = x;
                    if (ax < 0)
                        ax = width - 1;
                    if (ax > 300)
                        ax = 300;

                    out_neighIndex.emplace_back((ax * (polarNum + 1) + y) + z * (polarNum + 1) * (width + 1));
                }
            }
        }
    }

    /**
     * @brief delete clusters with fewer points, store clusters into a vector of point clouds
     * @param label_info, input category information
     * @return void
     */
    void labelAnalysis(std::vector<int> &label_info) {

        std::unordered_map<int, std::vector<int>> label2segIndex;
        size_t totalSize = label_info.size();
        for (size_t i = 0; i < totalSize; ++i) {
            // zero initialization for unordered_map
            label2segIndex[label_info[i]].emplace_back(i);
        }

        for (auto &it: label2segIndex) {
            if (it.second.size() >= params_.minSeg) {
                pcl::PointCloud<PointType>::Ptr cur_cloud(new pcl::PointCloud<PointType>);
                for (auto &idx: it.second) {
                    // cur_cloud->points.emplace_back(selected_points_->points[idx]);
                    cur_cloud->points.push_back(selected_points_->points[idx]);
                }
                clusters_.push_back(cur_cloud);
            }
        }
        // free memory
        std::unordered_map<int, std::vector<int>>().swap(label2segIndex);
    }

    void setParams(int semantic_class, double cluster_distance_threshold, int minNum, int maxNum,
                   clusterManager::ClusterParams &params) {
        params.semanticLabel = semantic_class;
        params.clusterTolerance = cluster_distance_threshold;
        params.minClusterSize = minNum;
        params.maxClusterSize = maxNum;
    }

    void setParams(int semantic_class, double cluster_distance_threshold, int minNum, int maxNum,
                   clusterManager::ClusterParams &params, clusterManager::DCVCParam &seg_param) {
        params.semanticLabel = semantic_class;
        params.clusterTolerance = cluster_distance_threshold;
        params.minClusterSize = minNum;
        params.maxClusterSize = maxNum;

        params.startR = seg_param.startR;
        params.deltaR = seg_param.deltaR;
        params.deltaP = seg_param.deltaP;
        params.deltaA = seg_param.deltaA;
        params.minSeg = seg_param.minSeg;
    }

    // feature method for background environmental semantics (e.g. Buildings)
    bool fpfh_feature_extraction(pcl::PointCloud<PointL>::Ptr input_src_sem_cloud,
                                 pcl::PointCloud<PointL>::Ptr input_tgt_sem_cloud) {
        // pointCloudL -> pointCloud
        if (!selectSemanticPoints(input_src_sem_cloud, input_tgt_sem_cloud)) {
            ROS_ERROR("Select semantic points failed! (in fpfh computation)");
        }

        // voxelize
        pcl::PointCloud<PointType>::Ptr srcFeat(new pcl::PointCloud<PointType>);
        pcl::PointCloud<PointType>::Ptr tgtFeat(new pcl::PointCloud<PointType>);

        voxelize(selected_src_points_, srcFeat, 0.5);
        voxelize(selected_tgt_points_, tgtFeat, 0.5);

        if (srcFeat->points.size() == 0 || tgtFeat->points.size() == 0) {
            return false;
        }

        pcl::PointCloud<pcl::Normal>::Ptr src_normals_raw(new pcl::PointCloud<pcl::Normal>);
        pcl::PointCloud<pcl::Normal>::Ptr tgt_normals_raw(new pcl::PointCloud<pcl::Normal>);

        static teaser::FPFHEstimation fpfh;

        pcl2teaser(*srcFeat, src_cloud_);
        pcl2teaser(*tgtFeat, tgt_cloud_);

        src_descriptors_ = fpfh.computeFPFHFeatures(src_cloud_, *src_normals_raw, 1, 2.5);
        tgt_descriptors_ = fpfh.computeFPFHFeatures(tgt_cloud_, *tgt_normals_raw, 1, 2.5);

        teaser::Matcher matcher;
        corr_ = matcher.calculateCorrespondences(
                src_cloud_, tgt_cloud_, *src_descriptors_, *tgt_descriptors_, true, true, true, 0.95);

        src_matched_.resize(3, corr_.size());
        tgt_matched_.resize(3, corr_.size());

        for (size_t i = 0; i < corr_.size(); ++i) {
            auto src_idx = std::get<0>(corr_[i]);
            auto dst_idx = std::get<1>(corr_[i]);
            src_matched_.col(i) << src_cloud_[src_idx].x, src_cloud_[src_idx].y, src_cloud_[src_idx].z;
            tgt_matched_.col(i) << tgt_cloud_[dst_idx].x, tgt_cloud_[dst_idx].y, tgt_cloud_[dst_idx].z;
        }

        eigen2pcl(src_matched_, src_matched_pcl_);
        eigen2pcl(tgt_matched_, tgt_matched_pcl_);

        // compute cov for src feature points
        static pcl::KdTreeFLANN<PointType> src_tree;
        static pcl::KdTreeFLANN<PointType> tgt_tree;
        src_tree.setInputCloud(srcFeat);
        tgt_tree.setInputCloud(tgtFeat);

        for (size_t i = 0; i < corr_.size(); ++i) {
            PointType temp_src, temp_tgt;
            temp_src = src_matched_pcl_.points[i];
            temp_tgt = tgt_matched_pcl_.points[i];

            std::vector<int> src_neighbor_index, tgt_neighbor_index;
            std::vector<float> src_dist, tgt_dist;

            int size_src = src_tree.radiusSearch(temp_src, 1, src_neighbor_index, src_dist);
            computeCov(srcFeat, src_neighbor_index, covariances_src_);
            int size_tgt = tgt_tree.radiusSearch(temp_tgt, 1, tgt_neighbor_index, tgt_dist);
            computeCov(tgtFeat, tgt_neighbor_index, covariances_tgt_);
        }

        return true;
    }

    pcl::PointCloud<PointType>::Ptr getSrcMatchedPointCloud() {
        return src_matched_pcl_.makeShared();
    }

    pcl::PointCloud<PointType>::Ptr getTgtMatchedPointCloud() {
        return tgt_matched_pcl_.makeShared();
    }

    std::vector<Eigen::Matrix3d> getSrcCovMat() {
        return covariances_src_;
    }

    std::vector<Eigen::Matrix3d> getTgtCovMat() {
        return covariances_tgt_;
    }

private:

    ClusterParams params_;

    // DBScan/Hierarchical DBScan related
    pcl::PointCloud<PointType>::Ptr selected_points_;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_;
    std::vector<pcl::PointIndices> cluster_indices_;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec_;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters_;

    // curved voxelization related
    double minPitch{0.0};
    double maxPitch{0.0};
    double minPolar{5.0};
    double maxPolar{5.0};
    int width{0};
    int height{0};
    // int minSeg{0};
    int polarNum{0};
    std::vector<double> polarBounds{};
    std::vector<Eigen::Vector3d> polarCor;
    std::unordered_map<int, std::vector<int>> voxelMap{};

    // fpfh related
    pcl::PointCloud<PointType>::Ptr selected_src_points_;
    pcl::PointCloud<PointType>::Ptr selected_tgt_points_;

    teaser::PointCloud src_cloud_, tgt_cloud_;
    teaser::FPFHCloudPtr src_descriptors_;
    teaser::FPFHCloudPtr tgt_descriptors_;
    std::vector<std::pair<int, int>> corr_; // Correspondence

    Eigen::Matrix<double, 3, Eigen::Dynamic> src_matched_; // matched fpfh feature points
    Eigen::Matrix<double, 3, Eigen::Dynamic> tgt_matched_;

    pcl::PointCloud<PointType> src_matched_pcl_;
    pcl::PointCloud<PointType> tgt_matched_pcl_;

    std::vector<Eigen::Matrix3d> covariances_src_;
    std::vector<Eigen::Matrix3d> covariances_tgt_;

    // cov mat computing relateds
    RegularizationMethod reg_method_;

};

#endif