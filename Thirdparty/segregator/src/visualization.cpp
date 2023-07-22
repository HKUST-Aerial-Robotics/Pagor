//
// Created by qzj on 23-2-22.
//

#include "visualization.h"
#include <eigen_conversions/eigen_msg.h>


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

pcl::PointCloud<PointType>::ConstPtr getCloud(std::string filename) {
    FILE *file = fopen(filename.c_str(), "rb");
    if (!file) {
        std::cerr << "error: failed to load point cloud " << filename << std::endl;
        return nullptr;
    }

    std::vector<float> buffer(1000000);
    size_t num_points =
            fread(reinterpret_cast<char *>(buffer.data()), sizeof(float), buffer.size(), file) / 4;
    fclose(file);

    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
    cloud->resize(num_points);

    for (int i = 0; i < num_points; i++) {
        auto &pt = cloud->at(i);
        pt.x = buffer[i * 4];
        pt.y = buffer[i * 4 + 1];
        pt.z = buffer[i * 4 + 2];
    }

    return cloud;
}

/**
* @brief      Merge cloud and label to semantic_pc.
* @param[in]  label_file_path; raw_point_cloud; out_semantic_pc
* @return     None.
*/
void merge_label(const string label_file_path,
                 pcl::PointCloud<PointType>::Ptr raw_pc,
                 pcl::PointCloud<PointL>::Ptr semantic_pc,
                 double label_deter_rate) {

    // read label file
    std::ifstream in_stream(label_file_path.c_str(), std::ios::in | std::ios::binary);
    vector <uint16_t> cloud_label_vec;
    cloud_label_vec.reserve(1000000);

    if (in_stream.is_open()) {
        // with 16 lower bit semantic label, 16 higher bit instance label
        uint32_t cur_whole_label;
        uint16_t cur_sem_label;

        while (in_stream.read((char *) &cur_whole_label, sizeof(cur_whole_label))) {
            cur_sem_label = cur_whole_label & 0xFFFF;
            cloud_label_vec.emplace_back(cur_sem_label);
        }
    } else {
        std::cerr << "error: failed to load label " << label_file_path << std::endl;
        return;
    }

    //check size equal
    if (raw_pc->points.size() != cloud_label_vec.size()) {
        std::cerr << "error: Point cloud size != label size" << std::endl;
        std::cout << "Point cloud size: " << raw_pc->points.size() << std::endl;
        std::cout << "Label size      : " << cloud_label_vec.size() << std::endl;
        return;
    }

    for (int i = 0; i < cloud_label_vec.size(); i++) {
        double cur_rand = (double) rand() / (RAND_MAX);
        if (cur_rand <= label_deter_rate) {
            cloud_label_vec[i] = 20;
        }
    }

    // semantic_pc.reset(new pcl::PointCloud<PointL>);
    for (int i = 0; i < raw_pc->points.size(); i++) {
        PointL tmpL;
        tmpL.x = raw_pc->points[i].x;
        tmpL.y = raw_pc->points[i].y;
        tmpL.z = raw_pc->points[i].z;
        tmpL.label = cloud_label_vec[i];
        semantic_pc->points.push_back(tmpL);
    }

    semantic_pc->width = semantic_pc->points.size();
    semantic_pc->height = 1;
}


void apply_color_mapping_spvnas(int label, int &r, int &g, int &b) {
    switch (label) {
        case 0: //car
        {
            r = 100;
            g = 150;
            b = 245;
            break;
        }
        case 1: //bicycle
        {
            r = 100;
            g = 230;
            b = 245;
            break;
        }
        case 2: //motorcycle
        {
            r = 30;
            g = 60;
            b = 150;
            break;
        }
        case 3: //truck
        {
            r = 80;
            g = 30;
            b = 180;
            break;
        }
        case 4: //other-vehicle
        {
            r = 0;
            g = 0;
            b = 255;
            break;
        }
        case 5: //person
        {
            r = 255;
            g = 30;
            b = 30;
            break;
        }
        case 6: //bicyclist
        {
            r = 255;
            g = 40;
            b = 200;
            break;
        }
        case 7: //motorcyclist
        {
            r = 150;
            g = 30;
            b = 90;
            break;
        }
        case 8: //road
        {
            r = 255;
            g = 0;
            b = 255;
            break;
        }
        case 9: //parking
        {
            r = 255;
            g = 150;
            b = 255;
            break;
        }
        case 10: //sidewalk
        {
            r = 75;
            g = 0;
            b = 75;
            break;
        }
        case 11: //other-ground
        {
            r = 175;
            g = 0;
            b = 75;
            break;
        }
        case 50: //building
        {
            r = 255;
            g = 200;
            b = 0;
            break;
        }
        case 13: //fence
        {
            r = 255;
            g = 120;
            b = 50;
            break;
        }
        case 14: //vegetation
        {
            r = 0;
            g = 175;
            b = 0;
            break;
        }
        case 15: //trunk
        {
            r = 135;
            g = 60;
            b = 0;
            break;
        }
        case 16: //terrain
        {
            r = 150;
            g = 240;
            b = 80;
            break;
        }
        case 17: //pole
        {
            r = 255;
            g = 240;
            b = 150;
            break;
        }
        case 18: //traffic-sign
        {
            r = 255;
            g = 0;
            b = 0;
            break;
        }
        default: //moving objects
        {
            r = 0;
            g = 0;
            b = 0;
        }
    }
}

void apply_color_mapping_salsanext(int label, int &r, int &g, int &b) {
    switch (label) {
        case 10: //car
        {
            r = 100;
            g = 150;
            b = 245;
            break;
        }
        case 1: //bicycle
        {
            r = 100;
            g = 230;
            b = 245;
            break;
        }
        case 2: //motorcycle
        {
            r = 30;
            g = 60;
            b = 150;
            break;
        }
        case 3: //truck
        {
            r = 80;
            g = 30;
            b = 180;
            break;
        }
        case 4: //other-vehicle
        {
            r = 0;
            g = 0;
            b = 255;
            break;
        }
        case 5: //person
        {
            r = 255;
            g = 30;
            b = 30;
            break;
        }
        case 6: //bicyclist
        {
            r = 255;
            g = 40;
            b = 200;
            break;
        }
        case 7: //motorcyclist
        {
            r = 150;
            g = 30;
            b = 90;
            break;
        }
        case 40: //road
        {
            r = 255;
            g = 0;
            b = 255;
            break;
        }
        case 9: //parking
        {
            r = 255;
            g = 150;
            b = 255;
            break;
        }
        case 48: //sidewalk
        {
            r = 75;
            g = 0;
            b = 75;
            break;
        }
        case 11: //other-ground
        {
            r = 175;
            g = 0;
            b = 75;
            break;
        }
        case 50: //building
        {
            r = 255;
            g = 200;
            b = 0;
            break;
        }
        case 13: //fence
        {
            r = 255;
            g = 120;
            b = 50;
            break;
        }
        case 70: //vegetation
        {
            r = 0;
            g = 175;
            b = 0;
            break;
        }
        case 18: //trunk
        {
            r = 135;
            g = 60;
            b = 0;
            break;
        }
        case 16: //terrain
        {
            r = 150;
            g = 240;
            b = 80;
            break;
        }
        case 17: //pole
        {
            r = 255;
            g = 240;
            b = 150;
            break;
        }
            //case 18: //traffic-sign
            //{
            //    r = 255;
            //    g = 0;
            //    b = 0;
            //    break;
            //}
        default: //moving objects
        {
            r = 0;
            g = 0;
            b = 0;
        }
    }
}

/**
* @brief      Color point cloud according to per point semantic labels.
* @param[in]  semantic_cloud: input semantic cloud ptr (with label)
* @param[in]  colored_cloud:  colored cloud ptr
*/
void color_pc(const pcl::PointCloud<PointL>::Ptr semantic_cloud, pcl::PointCloud<PointRGB>::Ptr colored_cloud,
              bool lift_z) {
    int r, g, b;
    uint16_t temp_label;
    PointRGB temp_pt;
    for (int i = 0; i < semantic_cloud->points.size(); ++i) {
        temp_pt.x = semantic_cloud->points[i].x;
        temp_pt.y = semantic_cloud->points[i].y;
        temp_pt.z = semantic_cloud->points[i].z;
        if (lift_z) {
            temp_pt.z += 10;
        }
        temp_label = semantic_cloud->points[i].label;
        //apply_color_mapping_spvnas((int)temp_label, r, g, b);
        apply_color_mapping_salsanext((int) temp_label, r, g, b);
        temp_pt.r = r;
        temp_pt.g = g;
        temp_pt.b = b;
        colored_cloud->points.push_back(temp_pt);
    }
}

void setCovMatsMarkers(visualization_msgs::MarkerArray &markerArray,
                       const pcl::PointCloud<PointType>::Ptr cloud,
                       const std::vector <Eigen::Matrix3d> &covariances,
                       const std::vector<float> rgb_color = {0.0, 0.0, 0.0},
                       int id = 0) {
    // int id = 1;
    Eigen::EigenSolver <Eigen::Matrix3d> es;
    for (int i = 0; i < covariances.size(); ++i) {
        visualization_msgs::Marker covMarker;

        covMarker.header.frame_id = "map";
        covMarker.header.stamp = ros::Time();
        covMarker.ns = "my_namespace";
        covMarker.id = id; // To avoid overlap
        covMarker.type = visualization_msgs::Marker::CYLINDER;
        covMarker.action = visualization_msgs::Marker::ADD;

        PointType tempP = cloud->points[i];
        covMarker.pose.position.x = tempP.x;
        covMarker.pose.position.y = tempP.y;
        covMarker.pose.position.z = tempP.z;

        es.compute(covariances[i], true);
        covMarker.scale.x = sqrt(es.eigenvalues()(0).real());
        covMarker.scale.y = sqrt(es.eigenvalues()(1).real());
        covMarker.scale.z = sqrt(es.eigenvalues()(2).real());

        covMarker.color.r = rgb_color[0];
        covMarker.color.g = rgb_color[1];
        covMarker.color.b = rgb_color[2];
        covMarker.color.a = 1.0; // Don't forget to set the alpha!

        Eigen::Matrix3d eigen_mat = es.eigenvectors().real();
        Eigen::Matrix3d rot_mat = eigen_mat.transpose();
        // eigen_mat.normalize();
        Eigen::Quaterniond quat(rot_mat);
        quat.normalize();

        geometry_msgs::Quaternion geo_quat;
        tf::quaternionEigenToMsg(quat, geo_quat);

        covMarker.pose.orientation.x = geo_quat.x;
        covMarker.pose.orientation.y = geo_quat.y;
        covMarker.pose.orientation.z = geo_quat.z;
        covMarker.pose.orientation.w = geo_quat.w;

        markerArray.markers.push_back(covMarker);
        id++;
    }
}

pcl::PointCloud<PointL>::Ptr random_downsample_pl(pcl::PointCloud<PointL>::Ptr cloud_ori, int ratio) {
    pcl::PointCloud<PointL>::Ptr sampled_pc(new pcl::PointCloud <PointL>);

    for (int i = 0; i < cloud_ori->points.size(); ++i) {
        if (i % ratio == 0) {
            sampled_pc->points.push_back(cloud_ori->points[i]);
        }
    }

    return sampled_pc;
}
