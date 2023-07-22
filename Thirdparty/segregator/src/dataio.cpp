//
// Created by qzj on 23-2-22.
//
#include "dataio.hpp"

using namespace std;

bool save_problematic_frames(std::ofstream &out, int src_index, int tgt_index) {

    if (!out) {
        return false;
    }

    std::string space_delimiter = " ";
    out << src_index << space_delimiter << tgt_index << std::endl;

    return true;
}

bool save_timing(std::ofstream &out, std::vector<double> time_vec, int src_index, int tgt_index) {
    if (!out) {
        return false;
    }
    std::string space_delimiter = " ";

    out << src_index << space_delimiter << tgt_index << space_delimiter;

    for (size_t i = 0; i < time_vec.size(); ++i) {
        if (i == time_vec.size() - 1) {
            out << time_vec[i];

        } else {
            out << time_vec[i] << space_delimiter;
        }
    }
    out << std::endl;

    return true;
}

bool
save_pose_error(std::ofstream &out, std::vector<double> e_t, std::vector<double> e_r, int src_index, int tgt_index) {
    if (!out) {
        return false;
    }
    if (e_t.size() != e_r.size()) {
        return false;
    }
    std::string space_delimiter = " ";

    out << src_index << space_delimiter << tgt_index << space_delimiter;

    for (size_t i = 0; i < e_t.size(); ++i) {
        if (i == e_t.size() - 1) {
            out << e_t[i] << space_delimiter << e_r[i];

        } else {
            out << e_t[i] << space_delimiter << e_r[i] << space_delimiter;
        }
    }
    out << std::endl;

    return true;
}

bool read_index_list(std::string index_file_path, std::vector<int> &source_indx, std::vector<int> &target_indx) {

    std::ifstream in(index_file_path, ios::in);

    if (!in) {
        return 0;
    }

    while (!in.eof()) {
        int p1, p2;
        in >> p1 >> p2;

        if (in.fail()) {
            break;
        }
        source_indx.emplace_back(p1);
        target_indx.emplace_back(p2);
    }

    return 1;
}

std::vector <Eigen::Matrix4d>
load_poses_from_transform_matrix(const std::string filepath) {
    double tmp[12];
    std::vector <Eigen::Matrix4d> pose_vec;
    Eigen::Matrix4d temp_pose = Eigen::Matrix4d::Identity();
    std::ifstream posereader(filepath);

    int count = 0;
    while (posereader >> tmp[0]) {
        for (int i = 1; i < 12; ++i) {
            posereader >> tmp[i];
        }
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 4; ++k) {
                temp_pose(j, k) = tmp[4 * j + k];
            }
        }

        pose_vec.push_back(temp_pose);

        count++;
        //LOG(WARNING) << temp_pose;
    }
    return pose_vec;
}

bool load_calib_mat(const std::string calib_file, Eigen::Matrix4d &calib_mat) {

    calib_mat.setIdentity();

    std::ifstream calibreader(calib_file);
    //cout<<filepath<<"\n";
    if (calibreader.is_open()) {
        while (calibreader.peek() != EOF) {
            std::string line;
            getline(calibreader, line);

            std::stringstream ss(line);

            std::string flag;

            ss >> flag;

            if (flag.compare("Tr:") == 0) {
                ss >> calib_mat(0, 0) >> calib_mat(0, 1) >> calib_mat(0, 2) >> calib_mat(0, 3) >> calib_mat(1, 0)
                   >> calib_mat(1, 1) >> calib_mat(1, 2) >> calib_mat(1, 3) >> calib_mat(2, 0) >> calib_mat(2, 1)
                   >> calib_mat(2, 2) >> calib_mat(2, 3);
                break;
            }
        }
        calibreader.close();
    } else
        return 0;

    // LOG(INFO) << "Calib matrix loaded\n";

    // std::cout << setprecision(16) << calib_mat << std::endl;

    return 1;
}

std::string kitti_zero_padding(int &file_name) {
    std::ostringstream ss;
    ss << std::setw(6) << std::setfill('0') << file_name;
    std::string s2(ss.str());
    return s2;
}