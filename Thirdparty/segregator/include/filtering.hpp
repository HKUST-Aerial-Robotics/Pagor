// adapted from QRATRO

//
// Created by Hyungtae Lim on 1/24/22.
// Our code is based on TEASER++. We really appreciate Prof. Luca Carlone group! :)
//

#ifndef FILTERING_H
#define FILTERING_H


#include <unistd.h>
#include <geometry_msgs/Pose.h>
#include <iostream>

#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/default_convergence_criteria.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <pcl/registration/transformation_estimation_svd.h>

//fpfh.h
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>

//pcl voxelgrid
#include <pcl/filters/voxel_grid.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "teaser/graph.h"

#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Geometry>

#include <flann/flann.hpp>
#include <fpfh_manager.hpp>

#include "teaser/utils.h"

#include <pcl/PCLPointCloud2.h>
#include "conversion.hpp"


using namespace std;
using namespace pcl;


template<typename T>
void voxelize(
        const boost::shared_ptr<pcl::PointCloud<T> > srcPtr, boost::shared_ptr<pcl::PointCloud<T> > dstPtr,
        double voxelSize) {
    static pcl::VoxelGrid<T> voxel_filter;
    voxel_filter.setInputCloud(srcPtr);
    voxel_filter.setLeafSize(voxelSize, voxelSize, voxelSize);
    voxel_filter.filter(*dstPtr);
}


template<typename T>
void voxelize(
        pcl::PointCloud<T> &src, boost::shared_ptr<pcl::PointCloud<T> > dstPtr,
        double voxelSize) {
    static pcl::VoxelGrid<T> voxel_filter;
    voxel_filter.setInputCloud(src);
    voxel_filter.setLeafSize(voxelSize, voxelSize, voxelSize);
    voxel_filter.filter(*dstPtr);
}


#endif //FILTERING_H