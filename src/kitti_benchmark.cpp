//
// Created by qzj on 2021/12/24.
//

#include "kitti_benchmark.h"
#include "glog/logging.h"
#include <ros/package.h>

namespace pagor {

    kitti_benchmark::kitti_benchmark(const std::string &config_name, int downsample) {

        std::string project_path = ros::package::getPath("pagor");
        std::string config_path = project_path + "/configs/" + config_name;
        LOG(INFO) << "config_path: " << config_path;
        config_node_ = YAML::LoadFile(config_path);
        kitti_loader_ptr_ = std::make_shared<KittiLoader>(config_path, downsample);
        LOG(INFO) << "KittiLoader initialized";
        segregator_ptr_ = std::make_shared<Segregator>(config_path, false);
        LOG(INFO) << "Segregator initialized";
        Run();
    }

    void kitti_benchmark::Run() {
        std::map<int, PoseArray> &poses_map = kitti_loader_ptr_->poses_map;
        std::map<int, std::map<std::string, std::vector<std::pair<int, int>>>> &data_map = kitti_loader_ptr_->data_map;
        std::map<int, std::vector<std::pair<string, double>>> result_map;
        for (auto &data: data_map) {
            int seq = data.first;
            std::map<std::string, std::vector<std::pair<int, int>>> indice_map = data.second;
            LOG(INFO) << "Sequence: " << seq;
            for (auto &item: indice_map) {
                std::string hardness = item.first;
                if (hardness != "hard") {
                    continue;
                }
                LOG(INFO) << "Hardness: " << hardness;
                std::vector<std::pair<int, int>> pair_idxes = item.second;
                int total = 0, success = 0, success_vote = 0;
                double total_time = 0, total_feature_time = 0, total_reg_time = 0, total_vote_time = 0;
                bool success_flag = false, success_flag_vote = false;
                for (auto &pair: pair_idxes) {
                    total++;
                    int start = pair.first;
                    int end = pair.second;
                    Eigen::Matrix4d pose_gt = poses_map[seq][end].inverse() * poses_map[seq][start];
                    std::string src_path = kitti_loader_ptr_->GetPCDPath(seq, start);
                    std::string tgt_path = kitti_loader_ptr_->GetPCDPath(seq, end);
                    std::string src_label_path = kitti_loader_ptr_->GetLabelPath(seq, start);
                    std::string tgt_label_path = kitti_loader_ptr_->GetLabelPath(seq, end);
                    //
                    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
                    std::chrono::steady_clock::time_point t2, t3, t4, t5;
                    clipper::CertifiedTransformations pose_candidates;
                    Eigen::Matrix4d best_pose = Eigen::Matrix4d::Identity();
                    if (segregator_ptr_->use_clipper) {
                        segregator_ptr_->reset(0.2, 5);
                        segregator_ptr_->LoadPCDLabFromFiles(src_path, tgt_path, src_label_path,
                                                             tgt_label_path); // takes 11ms
                        t2 = std::chrono::steady_clock::now();
                        segregator_ptr_->GobalRegistration(pose_candidates);
                        t3 = std::chrono::steady_clock::now();
                        best_pose = segregator_ptr_->SelectBestPose(pose_candidates);

                        //LOG(INFO) << "best_pose: \n" << best_pose;
                        //LOG(INFO) << "pose_gt: \n" << pose_gt;
                    } else {
                        double noise_bound = 0.2;
                        double distribution_noise_level = 5;
                        segregator_ptr_->reset(noise_bound, distribution_noise_level);
                        segregator_ptr_->LoadPCDLabFromFiles(src_path, tgt_path, src_label_path,
                                                             tgt_label_path); // takes 11ms
                        t2 = std::chrono::steady_clock::now();
                        Eigen::Matrix4d pose_est = segregator_ptr_->GobalRegistration();
                        t3 = std::chrono::steady_clock::now();
                        best_pose = pose_est;
                    }
                    t4 = std::chrono::steady_clock::now();
                    double time_used =
                            std::chrono::duration_cast<std::chrono::duration<double>>(t4 - t1).count() * 1000;
                    total_time += time_used;
                    total_feature_time +=
                            std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
                    total_reg_time += std::chrono::duration_cast<std::chrono::duration<double>>(t3 - t2).count() * 1000;
                    total_vote_time +=
                            std::chrono::duration_cast<std::chrono::duration<double>>(t4 - t3).count() * 1000;
                    //LOG(INFO) << "Load time: " << std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000 << "ms";
                    //LOG(INFO) << "Solver time: " << std::chrono::duration_cast<std::chrono::duration<double>>(t3 - t2).count() * 1000 << "ms";

                    Eigen::Matrix4d pose_diff = best_pose.inverse() * pose_gt;
                    double distance = sqrt(pose_diff(0, 3) * pose_diff(0, 3) + pose_diff(1, 3) * pose_diff(1, 3) +
                                           pose_diff(2, 3) * pose_diff(2, 3));
                    Eigen::Matrix3d rot_diff = pose_diff.block<3, 3>(0, 0);
                    double angle = dcm2angle(rot_diff, true);
                    success_flag_vote = false;
                    if (abs(distance) <= 2 && abs(angle) <= 5) {
                        success_vote++;
                        success_flag_vote = true;
                    }

                    success_flag = false;
                    for (auto pose_est: pose_candidates) {
                        Eigen::Matrix4d pose_diff = pose_est.first.inverse() * pose_gt;
                        double distance = sqrt(pose_diff(0, 3) * pose_diff(0, 3) + pose_diff(1, 3) * pose_diff(1, 3) +
                                               pose_diff(2, 3) * pose_diff(2, 3));
                        Eigen::Matrix3d rot_diff = pose_diff.block<3, 3>(0, 0);
                        double angle = dcm2angle(rot_diff, true);
                        if (abs(distance) <= 2 && abs(angle) <= 5) {
                            success++;
                            success_flag = true;
                            break;
                        }
                    }

                    std::vector<bool> success_flags(pose_candidates.size(), false);
                    for (int cand_i = 0; cand_i < pose_candidates.size(); cand_i++) {
                        Eigen::Matrix4d pose_diff = pose_candidates[cand_i].first.inverse() * pose_gt;
                        double distance = sqrt(pose_diff(0, 3) * pose_diff(0, 3) + pose_diff(1, 3) * pose_diff(1, 3) +
                                               pose_diff(2, 3) * pose_diff(2, 3));
                        Eigen::Matrix3d rot_diff = pose_diff.block<3, 3>(0, 0);
                        double angle = dcm2angle(rot_diff, true);
                        if (abs(distance) <= 2 && abs(angle) <= 5) {
                            success_flags[cand_i] = true;
                        }
                    }

                    std::string success_str = "";
                    for (int cand_i = 0; cand_i < pose_candidates.size(); cand_i++) {
                        if (success_flags[cand_i]) {
                            success_str += "1";
                        } else {
                            success_str += "0";
                        }
                    }

                    LOG(INFO) << "seq: " << seq << " " << hardness << ", " << total << "/" << pair_idxes.size()
                              << ", best_s: " << (double) success / total << " s: " << (double) success_vote / total
                              << ", feature_time: " << total_feature_time / total << "ms" << ", reg_time: "
                              << total_reg_time / total << "ms" << ", vote_time: " << total_vote_time / total << "ms"
                              << ", pair: (" << start << ", " << end << ")"
                              << ", s: " << success_flag << ", s_v: " << success_flag_vote << ", " << success_str;
                    //break;
                }
                result_map[seq].push_back(std::make_pair(hardness, (double) success_vote / total));
                LOG(INFO) << "Seq: " << seq << " " << hardness << " Succ ratio: " << (double) success_vote / total
                          << " upper bound: " << (double) success / total << " Time: " << total_time / total << "ms"
                          << " Feature time: " << total_feature_time / total << "ms"
                          << " Reg time: " << total_reg_time / total << "ms"
                          << " Vote time: " << total_vote_time / total << "ms";
            }
        }
        // print result
        for (auto &result: result_map) {
            int seq = result.first;
            std::vector<std::pair<std::string, double>> hardness_result = result.second;
            for (auto &item: hardness_result) {
                std::string hardness = item.first;
                double succ_ratio = item.second;
                LOG(INFO) << "Seq: " << seq << " " << hardness << " Succ ratio: " << succ_ratio;
            }
        }
    }
}