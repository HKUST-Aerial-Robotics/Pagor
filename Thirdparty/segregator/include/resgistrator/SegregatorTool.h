//
// Created by qzj on 23-2-22.
//

#ifndef SRC_SEGREGATOR_TOOL_H
#define SRC_SEGREGATOR_TOOL_H

#include <ros/ros.h>
#include <ros/package.h>
#include <locale>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>
#include <eigen_conversions/eigen_msg.h>
#include <visualization_msgs/Marker.h>

#include "filtering.hpp"
#include "cluster_manager.hpp"
#include "semantic_teaser.hpp"
#include "dataio.hpp"
#include "Segregator.h"
#include "visualization.h"

class SegregatorTool : public Segregator {
public:
    ros::NodeHandle nh_;

    // publishers
    ros::Publisher SrcPublisher = nh_.advertise<sensor_msgs::PointCloud2>("/source", 100);
    ros::Publisher SrcColoredPublisher = nh_.advertise<sensor_msgs::PointCloud2>("/srccolored", 100);
    ros::Publisher SrcCarCenterPublisher = nh_.advertise<sensor_msgs::PointCloud2>("/src_car_nodes", 100);
    ros::Publisher SrcBuildingCenterPublisher = nh_.advertise<sensor_msgs::PointCloud2>("/src_building_nodes", 100);
    ros::Publisher SrcVegCenterPublisher = nh_.advertise<sensor_msgs::PointCloud2>("/src_veg_nodes", 100);
    ros::Publisher SrcTrunkPublisher = nh_.advertise<sensor_msgs::PointCloud2>("/src_trunk_nodes", 100);
    ros::Publisher SrcCovPublisher = nh_.advertise<visualization_msgs::MarkerArray>("/src_cov", 100);

    ros::Publisher TgtPublisher = nh_.advertise<sensor_msgs::PointCloud2>("/target", 100);
    ros::Publisher TgtColoredPublisher = nh_.advertise<sensor_msgs::PointCloud2>("/tgtcolored", 100);
    ros::Publisher TgtCarCenterPublisher = nh_.advertise<sensor_msgs::PointCloud2>("/tgt_car_nodes", 100);
    ros::Publisher TgtBuildingCenterPublisher = nh_.advertise<sensor_msgs::PointCloud2>("/tgt_building_nodes", 100);
    ros::Publisher TgtVegCenterPublisher = nh_.advertise<sensor_msgs::PointCloud2>("/tgt_veg_nodes", 100);
    ros::Publisher TgtTrunkPublisher = nh_.advertise<sensor_msgs::PointCloud2>("/tgt_trunk_nodes", 100);
    ros::Publisher TgtCovPublisher = nh_.advertise<visualization_msgs::MarkerArray>("/tgt_cov", 100);

    ros::Publisher InlierCorrPublisher = nh_.advertise<visualization_msgs::Marker>("/inlierCorres", 100);
    ros::Publisher InitalCorrPublisher = nh_.advertise<visualization_msgs::Marker>("/initCorres", 100);

    ros::Publisher TransformedPublisher = nh_.advertise<sensor_msgs::PointCloud2>("/transformed_pc", 100);

public:

    SegregatorTool(ros::NodeHandle nh, std::string config_file, bool is_visualize = false) :
            nh_(nh), Segregator(config_file, is_visualize) {
        nh_ = nh;
    }

    void PublishToRviz(Eigen::Matrix4d &transformation) {
        if (!visualize)
            return;
        // for visualization
        pcl::PointCloud<PointRGB>::Ptr srcColoredRaw(new pcl::PointCloud <PointRGB>);
        pcl::PointCloud<PointRGB>::Ptr tgtColoredRaw(new pcl::PointCloud <PointRGB>);

        color_pc(srcSemanticPc, srcColoredRaw, true);
        color_pc(tgtSemanticPc, tgtColoredRaw);

        sensor_msgs::PointCloud2 SrcMsg = cloud2msg(*srcRaw);
        sensor_msgs::PointCloud2 SrcColoredMsg = cloud2msg(*srcColoredRaw);
        //sensor_msgs::PointCloud2 SrcCarCenterMsg  = cloud2msg(*srcCarCloud);
        //sensor_msgs::PointCloud2 SrcBuildingCenterMsg = cloud2msg(*srcBuildingCloud);
        //sensor_msgs::PointCloud2 SrcVegCenterMsg  = cloud2msg(*srcVegetationCloud);
        //sensor_msgs::PointCloud2 SrcTrunkMsg      = cloud2msg(*srcTrunkCloud);

        sensor_msgs::PointCloud2 TgtMsg = cloud2msg(*tgtRaw);
        sensor_msgs::PointCloud2 TgtColoredMsg = cloud2msg(*tgtColoredRaw);
        //sensor_msgs::PointCloud2 TgtCarCenterMsg  = cloud2msg(*tgtCarCloud);
        //sensor_msgs::PointCloud2 TgtBuildingCenterMsg = cloud2msg(*tgtBuildingCloud);
        //sensor_msgs::PointCloud2 TgtVegCenterMsg  = cloud2msg(*tgtVegetationCloud);
        //sensor_msgs::PointCloud2 TgtTrunkMsg      = cloud2msg(*tgtTrunkCloud);

        //LOG(INFO) << "transformation: \n" << transformation;
        pcl::PointCloud<PointType>::Ptr transformed_src(new pcl::PointCloud <PointType>);
        pcl::transformPointCloud(*srcRaw, *transformed_src, transformation);
        sensor_msgs::PointCloud2 TransformedMsg = cloud2msg(*transformed_src);

        // correspondence visualization
        pcl::PointCloud <PointType> srcMaxClique;
        pcl::PointCloud <PointType> tgtMaxClique;
        if (!use_clipper) {
            semSolver_ptr->getMaxCliques(srcMaxClique, tgtMaxClique);
        } else {
            clipper_ptr->getMaxCliques(srcMaxClique, tgtMaxClique);
        }
        LOG(INFO) << "srcMaxClique size: " << srcMaxClique.size();
        visualization_msgs::Marker inlierCorrMarker;
        setCorrespondenceMarker(srcMaxClique, tgtMaxClique, inlierCorrMarker, 0.5, {0.0, 1.0, 0.0}, 0, true);

        pcl::PointCloud <PointType> srcMatched;
        pcl::PointCloud <PointType> tgtMatched;
        semSolver_ptr->getInitCorr(srcMatched, tgtMatched);
        visualization_msgs::Marker initalCorrMarker;
        setCorrespondenceMarker(srcMatched, tgtMatched, initalCorrMarker, 0.08, {1.0, 0.0, 0.0}, 0);
        //
        //visualization_msgs::MarkerArray srcCovMarker, tgtCovMarker;
        //setCovMatsMarkers(srcCovMarker, srcBuildingCloud, src_building_covariances, {0.0, 0.0, 1.0}, 5);
        //setCovMatsMarkers(srcCovMarker, tgtBuildingCloud, tgt_building_covariances, {1.0, 0.0, 0.0}, 1000);

        ros::Rate loop_rate(10);
        while (ros::ok()) {
            SrcPublisher.publish(SrcMsg);
            SrcColoredPublisher.publish(SrcColoredMsg);
            //SrcCarCenterPublisher.publish(SrcCarCenterMsg);
            //SrcBuildingCenterPublisher.publish(SrcBuildingCenterMsg);
            //SrcVegCenterPublisher.publish(SrcVegCenterMsg);
            //SrcTrunkPublisher.publish(SrcTrunkMsg);
            //SrcCovPublisher.publish(srcCovMarker);

            TgtPublisher.publish(TgtMsg);
            TgtColoredPublisher.publish(TgtColoredMsg);
            //TgtCarCenterPublisher.publish(TgtCarCenterMsg);
            //TgtBuildingCenterPublisher.publish(TgtBuildingCenterMsg);
            //TgtVegCenterPublisher.publish(TgtVegCenterMsg);
            //TgtTrunkPublisher.publish(TgtTrunkMsg);
            //TgtCovPublisher.publish(tgtCovMarker);

            InlierCorrPublisher.publish(inlierCorrMarker);
            //InitalCorrPublisher.publish(initalCorrMarker);

            TransformedPublisher.publish(TransformedMsg);
            // GICPAlignedPublisher.publish(GicpAlignedMsg);

            loop_rate.sleep();
        }
    }
};


#endif //SRC_SEGREGATOR_TOOL_H
