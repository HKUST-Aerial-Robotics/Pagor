//
// Created by qzj on 23-2-22.
//

#ifndef SRC_KITTI_LOADER_H
#define SRC_KITTI_LOADER_H

#include <yaml-cpp/yaml.h>
#include <map>
#include <string>
#include <utility>
#include <iostream>
#include <fstream>
#include "glog/logging.h"
#include "CommonFunc.h"
#include <ros/package.h>

typedef std::vector<Eigen::Matrix4d> PoseArray;

class KittiLoader {

public:
    std::string dataset_path, label_dir;
    YAML::Node config_node_;
    int downsample_ = 1;

    // <seq, <hardness, <pair_id, pair_id>>>
    std::map<int, std::map<std::string, std::vector<std::pair<int, int>>>> data_map;
    std::map<int, PoseArray> poses_map;

public:
    KittiLoader(std::string &config_path, int downsample) {
        downsample_ = downsample;
        config_node_ = YAML::LoadFile(config_path);
        dataset_path = config_node_["dataset"]["dataset_path"].as<std::string>();
        label_dir = config_node_["dataset"]["label_dir"].as<std::string>();
        std::string split_dir = config_node_["dataset"]["split_dir"].as<std::string>();
        split_dir = ros::package::getPath("pagor") + "/" + split_dir;
        LoadDataSplit(split_dir);
        LoadKittiPose();
    }

    void LoadDataSplit(std::string &split_dir) {
        std::vector<std::string> split_files = {"kitti00_3to5.txt", "kitti00_8to10.txt", "kitti00_10to15.txt",
                                                "kitti05_3to5.txt", "kitti05_8to10.txt", "kitti05_10to15.txt",
                                                "kitti06_3to5.txt", "kitti06_8to10.txt", "kitti06_10to15.txt",
                                                "kitti07_3to5.txt", "kitti07_8to10.txt", "kitti07_10to15.txt",
                                                "kitti08_3to5.txt", "kitti08_8to10.txt", "kitti08_10to15.txt"};
        for (int i = 0; i < split_files.size(); i++) {
            std::string split_file = split_dir + "/" + split_files[i];
            std::ifstream fin(split_file, std::ios::in);
            std::string line;
            std::string seq = split_files[i].substr(5, 2);
            int seq_id = std::stoi(seq);
            std::string hardness = split_files[i].substr(8, 1);
            if (hardness == "3")
                hardness = "easy";
            else if (hardness == "8")
                hardness = "medium";
            else if (hardness == "1")
                hardness = "hard";
            else
                LOG(FATAL) << "Wrong hardness: " << hardness;
            const int downsample_rate = 10;
            int cnt = 0;
            while (std::getline(fin, line)) {
                std::stringstream ss(line);
                int pair_id1, pair_id2;
                ss >> pair_id1 >> pair_id2;
                cnt++;
                if (downsample_ > 1 && cnt % downsample_ != 0)
                    continue;
                data_map[seq_id][hardness].emplace_back(pair_id1, pair_id2);
            }
        }
        for(auto& seq : data_map){
            for(auto& hardness : seq.second){
                LOG(INFO) << "seq: " << seq.first << " hardness: " << hardness.first << " pair size: " << hardness.second.size();
            }
        }
    }

    Eigen::Matrix4d GetTr(std::string seq) {
        std::string calib_file = dataset_path + "/" + seq + "/calib.txt";
        std::fstream f;
        f.open(calib_file, std::ios::in);
        if (!f.is_open()) {
            LOG(FATAL) << "Cannot open calib file: " << calib_file;
        }
        std::string line;
        Eigen::Matrix4d Tr = Eigen::Matrix4d::Identity();
        while (std::getline(f, line)) {
            std::stringstream ss(line);
            std::string tag;
            ss >> tag;
            if (tag == "Tr:") {
                for (int i = 0; i < 3; ++i) {
                    for (int j = 0; j < 4; ++j) {
                        ss >> Tr(i, j);
                    }
                }
            }
        }
        return Tr;
    }

    void LoadKittiPose() {
        dataset_path = config_node_["dataset"]["dataset_path"].as<std::string>();
        std::vector<std::string> seqs = {"00", "05", "06", "07", "08"};
        for (int i = 0; i < seqs.size(); ++i) {
            std::string pose_file = dataset_path + "/" + seqs[i] + "/poses.txt";
            //    read kitti pose txt
            std::fstream f;
            f.open(pose_file, std::ios::in);
            if (!f.is_open()) {
                LOG(FATAL) << "Cannot open pose file: " << pose_file;
            }
            Eigen::Matrix4d Tr = GetTr(seqs[i]);
            std::string line;
            PoseArray poses;
            while (std::getline(f, line)) {
                std::stringstream ss(line);
                Eigen::Matrix4d Twc = Eigen::Matrix4d::Identity();
                for (int i = 0; i < 3; ++i) {
                    for (int j = 0; j < 4; ++j) {
                        ss >> Twc(i, j);
                    }
                }
                Eigen::Matrix4d Twl = Twc * Tr;
                poses.push_back(Twl);
            }
            poses_map[std::stoi(seqs[i])] = poses;
//            LOG(INFO) << "seq: " << seqs[i] << " pose size: " << poses.size();
        }
    }

    std::string GetPCDPath(int seq_id, int pair_id) {
        std::string seq = zfill(seq_id, 2);
        std::string pcd_path;
        pcd_path = dataset_path + "/" + seq + "/velodyne/" + zfill(pair_id, 6) + ".bin";
        return pcd_path;
    }

    std::string GetLabelPath(int seq_id, int pair_id) {
        std::string seq = zfill(seq_id, 2);
        std::string label_path;
        label_path = dataset_path + "/" + seq + label_dir + zfill(pair_id, 6) + ".label";
        return label_path;
    }
};


#endif //SRC_KITTI_LOADER_H
