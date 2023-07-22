//
// Created by qzj on 23-2-27.
//

#ifndef SRC_PCL_DIST_H
#define SRC_PCL_DIST_H

#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>
#include <cmath>
#include <chrono>
#include <glog/logging.h>
#include <execution>

template<typename PointT>
double TruncatedCD(typename pcl::PointCloud<PointT>::Ptr srcRaw, typename pcl::PointCloud<PointT>::Ptr tgtRaw,
                   const Eigen::Matrix4d &pose) {
    // transform the source point cloud
    typename pcl::PointCloud<PointT>::Ptr transformed_src(new pcl::PointCloud <PointT>);
    pcl::transformPointCloud(*srcRaw, *transformed_src, pose);
    // compute the chamfer distance
    double score = 0.0, dist;
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree <PointT>);
    tree->setInputCloud(tgtRaw);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    for (int i = 0; i < transformed_src->size(); ++i) {
        std::vector<int> pointIdxNKNSearch;
        std::vector<float> pointNKNSquaredDistance;
        tree->radiusSearch(transformed_src->points[i], 1.0, pointIdxNKNSearch, pointNKNSquaredDistance, 1);
        dist = pointIdxNKNSearch.size() == 0 ? 1.0 : pointNKNSquaredDistance[0];
        score += dist;
    }
    std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
    LOG(INFO) << "tree build time: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << "ms"
              << " search time: " << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count() << "ms"
              << std::endl;
    score = score / transformed_src->size();
    return score;
}

template<typename PointT>
double EnhancedTruncatedCD(typename pcl::PointCloud<PointT>::Ptr srcRaw, typename pcl::PointCloud<PointT>::Ptr tgtRaw,
                           const Eigen::Matrix4d &pose) {
    // transform the source point cloud
    typename pcl::PointCloud<PointT>::Ptr transformed_src(new pcl::PointCloud <PointT>);
    pcl::transformPointCloud(*srcRaw, *transformed_src, pose);
    // compute the chamfer distance
    double score = 0.0, dist;
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree <PointT>);
    tree->setInputCloud(tgtRaw);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);
    int match_num = 0;
    for (int i = 0; i < transformed_src->size(); ++i) {
        tree->nearestKSearch(transformed_src->points[i], 1, pointIdxNKNSearch, pointNKNSquaredDistance);
        dist = pointNKNSquaredDistance[0];
        if (dist < 1) {
            match_num++;
            score += dist;
        }
    }
    std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
    LOG(INFO) << "tree build time: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << "ms"
              << " search time: " << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count() << "ms"
              << std::endl;
    if (match_num == 0)
        return INFINITY;
    score = score / match_num * sqrt(transformed_src->size() / match_num);
    return score;
}

#endif //SRC_PCL_DIST_H
