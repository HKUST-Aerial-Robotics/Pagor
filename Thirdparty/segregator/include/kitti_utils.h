//
// Created by qzj on 23-2-26.
//

#ifndef SRC_KITTI_UTILS_H
#define SRC_KITTI_UTILS_H

namespace kitti_utils {

    inline std::string zfill(int num, int width) {
        std::stringstream ss;
        ss << std::setw(width) << std::setfill('0') << num;
        return ss.str();
    }

    inline Eigen::Matrix4d GetTr(const std::string &calib_path, const int &seq) {
        // calib_path: /media/qzj/Document/datasets/KITTI/odometry/data_odometry_calib/dataset/sequences
        std::string calib_file = calib_path + "/" + zfill(seq, 2) + "/calib.txt";
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

    inline Eigen::Matrix4d getLidarPose(const std::string &dataset_path, const std::string &calib_path,
                                        const int &seq, const int &frame_id) {
        // dataset_path: /media/qzj/Document/datasets/KITTI/odometry/data_odometry_velodyne/dataset/sequences
        // calib_path: /media/qzj/Document/datasets/KITTI/odometry/data_odometry_calib/dataset/sequences
        std::string pose_file = dataset_path + "/" + zfill(seq, 2) + "/poses.txt";
        //    read kitti pose txt
        std::fstream f;
        f.open(pose_file, std::ios::in);
        if (!f.is_open()) {
            LOG(FATAL) << "Cannot open pose file: " << pose_file;
        }
        Eigen::Matrix4d Tr = GetTr(calib_path, seq);
        std::string line;
        int num = 0;
        while (std::getline(f, line)) {
            if (num == frame_id) {
                std::stringstream ss(line);
                Eigen::Matrix4d Twc = Eigen::Matrix4d::Identity();
                for (int i = 0; i < 3; ++i) {
                    for (int j = 0; j < 4; ++j) {
                        ss >> Twc(i, j);
                    }
                }
                Eigen::Matrix4d Twl = Twc * Tr;
                return Twl;
            }
            num++;
        }
        return Eigen::Matrix4d::Identity();
    }

    inline std::string getPcdPath(const std::string &dataset_path, const int &seq, const int &frame_id) {
        // dataset_path: /media/qzj/Document/datasets/KITTI/odometry/data_odometry_velodyne/dataset/sequences
        std::string pcd_path = dataset_path + "/" + zfill(seq, 2) + "/velodyne/" + zfill(frame_id, 6) + ".bin";
        return pcd_path;
    }

    inline std::string getPcdLabelPath(const std::string &dataset_path, const int &seq, const int &frame_id,
                                       const std::string &label_dir) {
        // dataset_path: /media/qzj/Document/datasets/KITTI/odometry/data_odometry_velodyne/dataset/sequences
        std::string pcd_path = dataset_path + "/" + zfill(seq, 2) + label_dir + zfill(frame_id, 6) + ".label";
        return pcd_path;
    }
}

#endif //SRC_KITTI_UTILS_H


