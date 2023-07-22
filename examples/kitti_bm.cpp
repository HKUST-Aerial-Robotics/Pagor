#include <iostream>
#include <string>
#include "glog/logging.h"
#include "file_manager.hpp"
#include <Eigen/Core>
#include "kitti_benchmark.h"
#include <ros/package.h>

using namespace std;

void InitGlog(const std::string &project_path);

int main(int argc, char **argv) {

    //cout.rdbuf(NULL);
    std::string project_path = ros::package::getPath("pagor");
    InitGlog(project_path);
    LOG(INFO) << "project_path: " << project_path;
    std::string config_name = argv[1];
    int downsample = argc > 2 ? std::stoi(argv[2]) : false;
    pagor::kitti_benchmark kitti_bm(config_name, downsample);

    return 0;
}

void InitGlog(const std::string &project_path) {
    google::InitGoogleLogging("pagor");
    FLAGS_log_dir = project_path + "/Log";
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;
    FLAGS_log_prefix = false;
    FLAGS_logbufsecs = 0;
    FileManager::CreateDirectory(FLAGS_log_dir);
}
