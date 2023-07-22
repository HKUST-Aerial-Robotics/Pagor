//
// Created by qzj on 23-2-22.
//

#include "resgistrator/Segregator.h"
#include <pcl/filters/voxel_grid.h>
#include "resgistrator/pcl_dist.h"

const char separator = ' ';
const int nameWidth = 22;
const int numWidth = 8;

Eigen::Matrix4d Segregator::GobalRegistration() {

    std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
    TransToGaussian(srcSemanticPc, src_sem_vec, src_covariances);
    TransToGaussian(tgtSemanticPc, tgt_sem_vec, tgt_covariances);

    TransToGaussianBg(src_matched_bg, src_bg_covariances, tgt_matched_bg, tgt_bg_covariances);

    std::chrono::system_clock::time_point before_optim = std::chrono::system_clock::now();
    //LOG(INFO) << "Time for Gaussian transformation: " << std::chrono::duration_cast<std::chrono::milliseconds>(before_optim - start).count() << " ms";

    Eigen::Matrix4d solution;
    if (solving_w_cov) {
        semSolver_ptr->solve_for_multiclass_with_cov(src_sem_vec, tgt_sem_vec,
                                                     src_covariances, tgt_covariances,
                                                     *src_matched_bg, *tgt_matched_bg,
                                                     src_bg_covariances, src_bg_covariances);
        //semSolver_ptr->solve_for_multiclass_with_cov(src_sem_vec, tgt_sem_vec,
        //                                             src_covariances, tgt_covariances);
        //semSolver_ptr->solve_for_multiclass_with_cov(*src_matched_bg, *tgt_matched_bg,
        //                                        src_bg_covariances, src_bg_covariances);
    } else {
        //fixme: comparison is not rigorous enough, since fpfhs are not used in the solver
        semSolver_ptr->solve_for_multiclass(src_sem_vec, tgt_sem_vec);
    }
    solution = semSolver_ptr->get_solution();

    return solution;
}

void Segregator::GobalRegistration(clipper::CertifiedTransformations &solutions) {

    std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
    TransToGaussian(srcSemanticPc, src_sem_vec, src_covariances);
    TransToGaussian(tgtSemanticPc, tgt_sem_vec, tgt_covariances);

    TransToGaussianBg(src_matched_bg, src_bg_covariances, tgt_matched_bg, tgt_bg_covariances);

    std::chrono::system_clock::time_point before_optim = std::chrono::system_clock::now();
    //LOG(INFO) << "Time for Gaussian transformation: " << std::chrono::duration_cast<std::chrono::milliseconds>(before_optim - start).count() << " ms";

    Eigen::Matrix4d solution;
    clipper_ptr->solve_for_multiclass_with_cov(src_sem_vec, tgt_sem_vec,
                                               src_covariances, tgt_covariances,
                                               *src_matched_bg, *tgt_matched_bg,
                                               src_bg_covariances, src_bg_covariances);
    std::chrono::system_clock::time_point construct_c = std::chrono::system_clock::now();
    //LOG(INFO) << "Time for constructing clipper: " << std::chrono::duration_cast<std::chrono::milliseconds>(construct_c - before_optim).count() << " ms";
    clipper_ptr->solve();

#ifdef TEST_RUNTIME
    static double total_time_pose = 0;
    static double total_count_pose = 0;
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#endif
    clipper_ptr->getTransformationMatrix(solutions);
#ifdef TEST_RUNTIME
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    double time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
    total_time_pose += time_used;
    total_count_pose++;
    LOG(INFO) << "getTransformationMatrix time: " << time_used << " avg time: " << total_time_pose / total_count_pose << " ms";
#endif
}

void Segregator::check(const Eigen::Matrix4d &pose) {
    if (use_clipper) {
        clipper_ptr->check(pose);
    } else {
        //semSolver_ptr->check(pose);
    }
}

double Segregator::GetScore(const Eigen::Matrix4d &pose) {
    const double leaf_size = 1;
    pcl::PointCloud<PointType>::Ptr downsampled_src(new pcl::PointCloud <PointType>);
    pcl::PointCloud<PointType>::Ptr downsampled_tgt(new pcl::PointCloud <PointType>);
    pcl::VoxelGrid <PointType> sor;
    sor.setInputCloud(srcRaw);
    sor.setLeafSize(leaf_size, leaf_size, leaf_size);
    sor.filter(*downsampled_src);
    sor.setInputCloud(tgtRaw);
    sor.setLeafSize(leaf_size, leaf_size, leaf_size);
    sor.filter(*downsampled_tgt);
    pcl::PointCloud<PointType>::Ptr transformed_src(new pcl::PointCloud <PointType>);
    pcl::transformPointCloud(*downsampled_src, *transformed_src, pose);

    // compute the chamfer distance
    double score = 0.0, dist;
    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree <PointType>);
    tree->setInputCloud(downsampled_tgt);
    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);
    int match_num = 0;
    for (int i = 0; i < transformed_src->size(); ++i) {
        tree->nearestKSearch(transformed_src->points[i], 1, pointIdxNKNSearch, pointNKNSquaredDistance);
        dist = pointNKNSquaredDistance[0];
        //LOG(INFO) << "dist: " << dist << std::endl;
        if (dist < leaf_size * sqrt(3)) {
            score += dist;
            match_num++;
        }
    }
    score = score / match_num * sqrt(transformed_src->size() / match_num);
    return score;
}

double Segregator::GetScore2(const Eigen::Matrix4d &pose) {
    //LOG(INFO) << std::setprecision(4) << "pose:\n" << pose << std::endl;
    pcl::transformPointCloud(*srcRaw, *transformed_src, pose);
    // compute the chamfer distance
    double score = 0.0, dist;
    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree <PointType>);
    tree->setInputCloud(tgtRaw);
    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);
    for (int i = 0; i < transformed_src->size(); ++i) {
        tree->nearestKSearch(transformed_src->points[i], 1, pointIdxNKNSearch, pointNKNSquaredDistance);
        dist = pointNKNSquaredDistance[0];
        //LOG(INFO) << "dist: " << dist << std::endl;
        dist = min(dist, 1.0);
        score += dist;
    }
    score = score / transformed_src->size();

    return score;
}

Eigen::Matrix4d Segregator::SelectBestPose(const clipper::CertifiedTransformations &pose_candidates) {
    if (pose_candidates.size() == 1)
        return pose_candidates[0].first;

    // downsample the point cloud
    double leaf_size = 0.3;
    pcl::PointCloud<PointType>::Ptr downsampled_src(new pcl::PointCloud <PointType>);
    pcl::PointCloud<PointType>::Ptr downsampled_tgt(new pcl::PointCloud <PointType>);
    pcl::VoxelGrid <PointType> sor;
    sor.setInputCloud(srcRaw);
    sor.setLeafSize(leaf_size, leaf_size, leaf_size);
    sor.filter(*downsampled_src);
    sor.setInputCloud(tgtRaw);
    sor.setLeafSize(leaf_size, leaf_size, leaf_size);
    sor.filter(*downsampled_tgt);

    const double search_radius = 1.0;
    // compute the chamfer distance
    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree <PointType>);
    tree->setInputCloud(downsampled_tgt);
    Eigen::Matrix4d best_pose = Eigen::Matrix4d::Identity();
    double best_score = INFINITY;
    for (auto cer_trans: pose_candidates) {
        if (!cer_trans.second)
            continue;
        double score = 0.0, dist;
        Eigen::Matrix4d &pose = cer_trans.first;
        pcl::transformPointCloud(*downsampled_src, *transformed_src, pose);
        for (int i = 0; i < transformed_src->size(); ++i) {
            std::vector<int> pointIdxNKNSearch;
            std::vector<float> pointNKNSquaredDistance;
            tree->radiusSearch(transformed_src->points[i], search_radius, pointIdxNKNSearch, pointNKNSquaredDistance,
                               1);
            dist = pointIdxNKNSearch.size() == 0 ? search_radius : pointNKNSquaredDistance[0];
            score += dist;
        }
        score = transformed_src->size() > 0 ? score / transformed_src->size() : INFINITY;
        if (score < best_score) {
            best_score = score;
            best_pose = pose;
        }
    }
    return best_pose;
}


void Segregator::TransToGaussian(pcl::PointCloud<PointL>::Ptr semanticPc,
                                 std::vector <pcl::PointCloud<PointType>> &sem_vec,
                                 std::vector <Eigen::Matrix3d> &covariances) {

    //  clouds nodes
    pcl::PointCloud<PointType>::Ptr CarCloud(new pcl::PointCloud <PointType>);
    std::vector <Eigen::Matrix3d> car_covariances;
    if (use_car) {
        clusterManager car_node;
        car_node.reset(car_params);
        if (!use_DCVC_car) {
            car_node.dbscanSeg(semanticPc);
        } else {
            car_node.segmentPointCloud(semanticPc);
        }
        car_node.computeCentroidAndCov(CarCloud, car_covariances);
        sem_vec.emplace_back(*CarCloud);
        std::copy(std::begin(car_covariances), std::end(car_covariances), std::back_inserter(covariances));
    }

    pcl::PointCloud<PointType>::Ptr TrunkCloud(new pcl::PointCloud <PointType>);
    std::vector <Eigen::Matrix3d> trunk_covariances;
    if (use_trunk) {
        clusterManager trunk_node;
        trunk_node.reset(trunk_params);
        if (!use_DCVC_trunk) {
            trunk_node.dbscanSeg(semanticPc);
        } else {
            trunk_node.segmentPointCloud(semanticPc);
        }
        trunk_node.computeCentroidAndCov(TrunkCloud, trunk_covariances);
        sem_vec.emplace_back(*TrunkCloud);
        std::copy(std::begin(trunk_covariances), std::end(trunk_covariances), std::back_inserter(covariances));
    }
}

void Segregator::TransToGaussianBg(pcl::PointCloud<PointType>::Ptr src_matched_bg,
                                   std::vector <Eigen::Matrix3d> &src_bg_covariances,
                                   pcl::PointCloud<PointType>::Ptr tgt_matched_bg,
                                   std::vector <Eigen::Matrix3d> &tgt_bg_covariances) {

    pcl::PointCloud<PointType>::Ptr srcVegetationCloud(new pcl::PointCloud <PointType>);
    std::vector <Eigen::Matrix3d> src_veg_covariances;
    pcl::PointCloud<PointType>::Ptr tgtVegetationCloud(new pcl::PointCloud <PointType>);
    std::vector <Eigen::Matrix3d> tgt_veg_covariances;

    pcl::PointCloud<PointType>::Ptr srcBuildingCloud(new pcl::PointCloud <PointType>);
    std::vector <Eigen::Matrix3d> src_building_covariances;
    pcl::PointCloud<PointType>::Ptr tgtBuildingCloud(new pcl::PointCloud <PointType>);
    std::vector <Eigen::Matrix3d> tgt_building_covariances;

    if (use_building) {
        clusterManager building_node;
        building_node.reset(building_params);

        if (building_node.fpfh_feature_extraction(srcSemanticPc, tgtSemanticPc)) {
            srcBuildingCloud = building_node.getSrcMatchedPointCloud();
            tgtBuildingCloud = building_node.getTgtMatchedPointCloud();

            src_building_covariances = building_node.getSrcCovMat();
            tgt_building_covariances = building_node.getTgtCovMat();

            std::copy(std::begin(src_building_covariances), std::end(src_building_covariances),
                      std::back_inserter(src_bg_covariances));
            std::copy(std::begin(tgt_building_covariances), std::end(tgt_building_covariances),
                      std::back_inserter(tgt_bg_covariances));
        }
    }

    if (use_veg) {
        clusterManager veg_node;
        veg_node.reset(veg_params);

        if (veg_node.fpfh_feature_extraction(srcSemanticPc, tgtSemanticPc)) {
            srcVegetationCloud = veg_node.getSrcMatchedPointCloud();
            tgtVegetationCloud = veg_node.getTgtMatchedPointCloud();

            src_veg_covariances = veg_node.getSrcCovMat();
            tgt_veg_covariances = veg_node.getTgtCovMat();

            std::copy(std::begin(src_veg_covariances), std::end(src_veg_covariances),
                      std::back_inserter(src_bg_covariances));
            std::copy(std::begin(tgt_veg_covariances), std::end(tgt_veg_covariances),
                      std::back_inserter(tgt_bg_covariances));
        }
    }

    if (use_building || use_veg) {
        *src_matched_bg = (*srcBuildingCloud) + (*srcVegetationCloud);
        *tgt_matched_bg = (*tgtBuildingCloud) + (*tgtVegetationCloud);
    }
}