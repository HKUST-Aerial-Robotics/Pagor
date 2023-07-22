#ifndef _INCLUDE_DATA_IO_HPP
#define _INCLUDE_DATA_IO_HPP

//pcl
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

//boost
#include <boost/filesystem.hpp>
#include <boost/function.hpp>

//Eigen
#include <Eigen/Core>

// #include <glog/logging.h>
#include <string>
#include <fstream>
#include <vector>

bool save_problematic_frames(std::ofstream &out, int src_index, int tgt_index);

bool save_timing(std::ofstream &out, std::vector<double> time_vec, int src_index, int tgt_index);

bool
save_pose_error(std::ofstream &out, std::vector<double> e_t, std::vector<double> e_r, int src_index, int tgt_index);

bool read_index_list(std::string index_file_path, std::vector<int> &source_indx, std::vector<int> &target_indx);

std::vector<Eigen::Matrix4d>
load_poses_from_transform_matrix(const std::string filepath);

bool load_calib_mat(const std::string calib_file, Eigen::Matrix4d &calib_mat);

std::string kitti_zero_padding(int &file_name);


#endif // _INCLUDE_DATA_IO_HPP