//
// Created by qzj on 23-2-22.
//

#include "resgistrator/Segregator.h"
#include "visualization.h"
#include <pcl/io/pcd_io.h>
#include "file_manager.hpp"

Segregator::Segregator(std::string config_file, bool is_visualize) {
    visualize = is_visualize;
    LoadConfig(config_file);
    reset();
}

void Segregator::reset(double noise_level_custom, double distribution_noise_level_custom) {

    semanticTeaser::Params params;
    if (noise_level_custom < 0.0) {
        params.teaser_params.noise_bound = noise_level;
        params.teaser_params.distribution_noise_bound = distribution_noise_level;
    } else {
        params.teaser_params.noise_bound = noise_level_custom;
        params.teaser_params.distribution_noise_bound = distribution_noise_level_custom;
    }
    semSolver_ptr.reset(new semanticTeaser(params));

    clipper_ptr = std::make_shared<clipper::CLIPPER>(config_path_);

    // concatenate labels with point clouds
    srcRaw.reset(new pcl::PointCloud <PointType>);
    transformed_src.reset(new pcl::PointCloud <PointType>);
    tgtRaw.reset(new pcl::PointCloud <PointType>);
    srcSemanticPc.reset(new pcl::PointCloud <PointL>);
    tgtSemanticPc.reset(new pcl::PointCloud <PointL>);

    src_sem_vec.clear();
    src_covariances.clear();
    tgt_sem_vec.clear();
    tgt_covariances.clear();

    src_matched_bg.reset(new pcl::PointCloud <PointType>);
    src_bg_covariances.clear();
    tgt_matched_bg.reset(new pcl::PointCloud <PointType>);
    tgt_bg_covariances.clear();
}

void Segregator::LoadConfig(std::string config_file) {
    config_node_ = YAML::LoadFile(config_file);
    config_path_ = config_file;
    use_clipper = config_node_["solver"].as<std::string>() == "clipper";

    building_class_num = config_node_["building_param"]["class_num"].as<int>();
    building_min_cluster_dist = config_node_["building_param"]["min_dist"].as<double>();
    building_min_point_num = config_node_["building_param"]["min_num"].as<int>();
    building_max_point_num = config_node_["building_param"]["max_num"].as<int>();
    use_building = config_node_["building_param"]["use_building"].as<bool>();
    use_DCVC_building = config_node_["building_param"]["use_DCVC"].as<bool>();
    building_minSeg = config_node_["building_param"]["DCVC_min_num"].as<int>();

    car_class_num = config_node_["car_param"]["class_num"].as<int>();
    car_min_cluster_dist = config_node_["car_param"]["min_dist"].as<double>();
    car_min_point_num = config_node_["car_param"]["min_num"].as<int>();
    car_max_point_num = config_node_["car_param"]["max_num"].as<int>();
    use_car = config_node_["car_param"]["use_car"].as<bool>();
    use_DCVC_car = config_node_["car_param"]["use_DCVC"].as<bool>();
    car_minSeg = config_node_["car_param"]["DCVC_min_num"].as<int>();

    vegetation_class_num = config_node_["vegetation_param"]["class_num"].as<int>();
    vegetation_min_cluster_dist = config_node_["vegetation_param"]["min_dist"].as<double>();
    vegetation_min_point_num = config_node_["vegetation_param"]["min_num"].as<int>();
    vegetation_max_point_num = config_node_["vegetation_param"]["max_num"].as<int>();
    use_veg = config_node_["vegetation_param"]["use_veg"].as<bool>();
    use_DCVC_veg = config_node_["vegetation_param"]["use_DCVC"].as<bool>();
    veg_minSeg = config_node_["vegetation_param"]["DCVC_min_num"].as<int>();

    trunk_class_num = config_node_["trunk_param"]["class_num"].as<int>();
    trunk_min_cluster_dist = config_node_["trunk_param"]["min_dist"].as<double>();
    trunk_min_point_num = config_node_["trunk_param"]["min_num"].as<int>();
    trunk_max_point_num = config_node_["trunk_param"]["max_num"].as<int>();
    use_trunk = config_node_["trunk_param"]["use_trunk"].as<bool>();
    use_DCVC_trunk = config_node_["trunk_param"]["use_DCVC"].as<bool>();
    trunk_minSeg = config_node_["trunk_param"]["DCVC_min_num"].as<int>();

    startR = config_node_["DCVC_param"]["startR"].as<double>();
    deltaR = config_node_["DCVC_param"]["deltaR"].as<double>();
    deltaP = config_node_["DCVC_param"]["deltaP"].as<double>();
    deltaA = config_node_["DCVC_param"]["deltaA"].as<double>();
    minSeg = config_node_["DCVC_param"]["minSeg"].as<int>();

    noise_level = config_node_["noise_level"].as<double>();
    distribution_noise_level = config_node_["distribution_noise_level"].as<double>();

    src_indx = 0;
    tgt_indx = 4413;
    solving_w_cov = config_node_["solving_w_cov"].as<bool>();
    inital_yaw_rate = 0.0;
    label_deter_rate = 0.0;


    building_DCVC_param.startR = startR;
    building_DCVC_param.deltaR = deltaR;
    building_DCVC_param.deltaP = deltaP;
    building_DCVC_param.deltaA = deltaA;
    building_DCVC_param.minSeg = building_minSeg;

    car_DCVC_param.startR = startR;
    car_DCVC_param.deltaR = deltaR;
    car_DCVC_param.deltaP = deltaP;
    car_DCVC_param.deltaA = deltaA;
    car_DCVC_param.minSeg = car_minSeg;

    veg_DCVC_param.startR = startR;
    veg_DCVC_param.deltaR = deltaR;
    veg_DCVC_param.deltaP = deltaP;
    veg_DCVC_param.deltaA = deltaA;
    veg_DCVC_param.minSeg = veg_minSeg;

    trunk_DCVC_param.startR = startR;
    trunk_DCVC_param.deltaR = deltaR;
    trunk_DCVC_param.deltaP = deltaP;
    trunk_DCVC_param.deltaA = deltaA;
    trunk_DCVC_param.minSeg = trunk_minSeg;

    setParams(car_class_num, car_min_cluster_dist, car_min_point_num, car_max_point_num, car_params, car_DCVC_param);
    setParams(building_class_num, building_min_cluster_dist, building_min_point_num, building_max_point_num,
              building_params, building_DCVC_param);
    setParams(vegetation_class_num, vegetation_min_cluster_dist, vegetation_min_point_num, vegetation_max_point_num,
              veg_params, veg_DCVC_param);
    setParams(trunk_class_num, trunk_min_cluster_dist, trunk_min_point_num, trunk_max_point_num, trunk_params,
              trunk_DCVC_param);
}

void Segregator::LoadPCDLabFromFiles(std::string src_path, std::string tgt_path,
                                     std::string src_label_path, std::string tgt_label_path) {

    *srcRaw = *getCloud(src_path);
    *tgtRaw = *getCloud(tgt_path);

    merge_label(src_label_path, srcRaw, srcSemanticPc, -1);
    merge_label(tgt_label_path, tgtRaw, tgtSemanticPc, label_deter_rate);
}

void Segregator::SaveKeyPoints() {

    std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
    TransToGaussian(srcSemanticPc, src_sem_vec, src_covariances);
    TransToGaussian(tgtSemanticPc, tgt_sem_vec, tgt_covariances);

    TransToGaussianBg(src_matched_bg, src_bg_covariances, tgt_matched_bg, tgt_bg_covariances);

    std::string project_path = ros::package::getPath("segregator");
    std::string save_path = project_path + "/../data/";
    FileManager::CreateDirectory(save_path);
    std::string save_name;
    for (int i = 0; i < src_sem_vec.size(); ++i) {
        if (src_sem_vec[i].size() == 0 || tgt_sem_vec[i].size() == 0) {
            continue;
        }
        save_name = save_path + "src_sem_" + std::to_string(i) + ".pcd";
        pcl::io::savePCDFileASCII(save_name, src_sem_vec[i]);
        save_name = save_path + "tgt_sem_" + std::to_string(i) + ".pcd";
        pcl::io::savePCDFileASCII(save_name, tgt_sem_vec[i]);
        LOG(INFO) << "save src_sem_" << i << " size: " << src_sem_vec[i].size();
        LOG(INFO) << "save tgt_sem_" << i << " size: " << tgt_sem_vec[i].size();
    }
    if (src_matched_bg->size() == 0 || tgt_matched_bg->size() == 0) {
        return;
    }
    save_name = save_path + "src_bg.pcd";
    pcl::io::savePCDFileASCII(save_name, *src_matched_bg);
    save_name = save_path + "tgt_bg.pcd";
    pcl::io::savePCDFileASCII(save_name, *tgt_matched_bg);
    LOG(INFO) << "save src_bg size: " << src_matched_bg->size();
    LOG(INFO) << "save tgt_bg size: " << tgt_matched_bg->size();
}