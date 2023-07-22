#include "semantic_teaser.hpp"

void semanticTeaser::solve(const Eigen::MatrixX3d &src,
                           const Eigen::MatrixX3d &tgt) {

    culculate_correspondence(src, tgt);

    this->matched_src_ = this->initial_correspondences.points.first;
    this->matched_tgt_ = this->initial_correspondences.points.second;

    this->solver.solve(this->initial_correspondences.points.first,
                       this->initial_correspondences.points.second);

    // extract solution
    auto solution = this->solver.getSolution();
    this->solution_ = Eigen::Matrix4d::Identity();
    this->solution_.block<3, 3>(0, 0) = solution.rotation;
    this->solution_.block<3, 1>(0, 3) = solution.translation;

    this->max_clique_ = this->solver.getInlierMaxClique();
}

void semanticTeaser::solve_for_multiclass(const std::vector <pcl::PointCloud<pcl::PointXYZ>> &src_cloud_vec,
                                          const std::vector <pcl::PointCloud<pcl::PointXYZ>> &tgt_cloud_vec) {
    int total_corres_size = 0;
    int src_node_size = 0;
    int tgt_node_size = 0;
    for (int i = 0; i < src_cloud_vec.size(); i++) {
        src_node_size += src_cloud_vec[i].points.size();
        tgt_node_size += tgt_cloud_vec[i].points.size();
        total_corres_size = total_corres_size + src_cloud_vec[i].points.size() * tgt_cloud_vec[i].points.size();
    }

//    std::cout << "\033[1;31mSize sum up: " << "src " << src_node_size << "   tgt " << tgt_node_size << "corres " << total_corres_size << ".\033[0m" << std::endl;

    this->initial_correspondences.points = std::make_pair(Eigen::Matrix3Xd::Zero(3, total_corres_size),
                                                          Eigen::Matrix3Xd::Zero(3, total_corres_size));

    int class_num = src_cloud_vec.size();
    for (int indx = 0; indx < class_num; ++indx) {
        Eigen::MatrixX3d src_mat;
        Eigen::MatrixX3d tgt_mat;
        pcl2eigen(src_cloud_vec[indx], src_mat);
        pcl2eigen(tgt_cloud_vec[indx], tgt_mat);

        int index = 0;
        // build all to all correspondence
        for (int src_index = 0; src_index < src_mat.rows(); src_index++) {
            for (int tgt_index = 0; tgt_index < tgt_mat.rows(); tgt_index++) {
                this->initial_correspondences.points.first.col(index) << src_mat(src_index, 0), src_mat(src_index,
                                                                                                        1), src_mat(
                        src_index, 2);
                this->initial_correspondences.points.second.col(index) << tgt_mat(tgt_index, 0), tgt_mat(tgt_index,
                                                                                                         1), tgt_mat(
                        tgt_index, 2);
                this->initial_correspondences.indices.first.push_back(src_index);
                this->initial_correspondences.indices.second.push_back(tgt_index);
                index++;
            }
        }

    }

    this->matched_src_ = this->initial_correspondences.points.first;
    this->matched_tgt_ = this->initial_correspondences.points.second;

    this->solver.solve(this->initial_correspondences.points.first,
                       this->initial_correspondences.points.second);

    // extract solution
    auto solution = this->solver.getSolution();
    this->solution_ = Eigen::Matrix4d::Identity();
    this->solution_.block<3, 3>(0, 0) = solution.rotation;
    this->solution_.block<3, 1>(0, 3) = solution.translation;

    this->max_clique_ = this->solver.getInlierMaxClique();

}

void semanticTeaser::solve_for_multiclass_with_cov(const std::vector <pcl::PointCloud<pcl::PointXYZ>> &src_cloud_vec,
                                                   const std::vector <pcl::PointCloud<pcl::PointXYZ>> &tgt_cloud_vec,
                                                   const std::vector <Eigen::Matrix3d> &src_covariances,
                                                   const std::vector <Eigen::Matrix3d> &tgt_covariances,
                                                   const pcl::PointCloud <pcl::PointXYZ> &src_matched_cloud,
                                                   const pcl::PointCloud <pcl::PointXYZ> &tgt_matched_cloud,
                                                   const std::vector <Eigen::Matrix3d> &src_matched_covariances,
                                                   const std::vector <Eigen::Matrix3d> &tgt_matched_covariances) {
    int total_corres_size = 0;
    int src_node_size = 0;
    int tgt_node_size = 0;
    for (int i = 0; i < src_cloud_vec.size(); i++) {
        src_node_size += src_cloud_vec[i].points.size();
        tgt_node_size += tgt_cloud_vec[i].points.size();
        total_corres_size = total_corres_size + src_cloud_vec[i].points.size() * tgt_cloud_vec[i].points.size();
    }

    total_corres_size += src_matched_cloud.points.size();

//    std::cout << "\033[1;31mSize sum up: " << "src " << src_node_size << "   tgt " << tgt_node_size << "   corres " << total_corres_size << ".\033[0m" << std::endl;

    this->initial_correspondences.points = std::make_pair(Eigen::Matrix3Xd::Zero(3, total_corres_size),
                                                          Eigen::Matrix3Xd::Zero(3, total_corres_size));

    this->src_covariances_matched_.reserve(total_corres_size);
    this->tgt_covariances_matched_.reserve(total_corres_size);

    int index = 0;// index through the whole matched vector

    int class_num = src_cloud_vec.size();
    //LOG(INFO) << "src_cloud_vec.size() = " << src_cloud_vec.size();
    //LOG(INFO) << "tgt_cloud_vec.size() = " << tgt_cloud_vec.size();
    for (int class_indx = 0; class_indx < class_num; ++class_indx) {
        Eigen::MatrixX3d src_mat;
        Eigen::MatrixX3d tgt_mat;
        pcl2eigen(src_cloud_vec[class_indx], src_mat);
        pcl2eigen(tgt_cloud_vec[class_indx], tgt_mat);

        // build all to all correspondence
        for (int src_index = 0; src_index < src_mat.rows(); src_index++) {
            for (int tgt_index = 0; tgt_index < tgt_mat.rows(); tgt_index++) {
                this->initial_correspondences.points.first.col(index) << src_mat(src_index, 0), src_mat(src_index,
                                                                                                        1), src_mat(
                        src_index, 2);
                this->initial_correspondences.points.second.col(index) << tgt_mat(tgt_index, 0), tgt_mat(tgt_index,
                                                                                                         1), tgt_mat(
                        tgt_index, 2);
                this->initial_correspondences.indices.first.push_back(src_index);
                this->initial_correspondences.indices.second.push_back(tgt_index);
                this->src_covariances_matched_.emplace_back(src_covariances[src_index]);
                this->tgt_covariances_matched_.emplace_back(tgt_covariances[tgt_index]);
                // this->tgt_covariances_matched_[index] = tgt_covariances[tgt_index];
                index++;
            }
        }
    }

    // append matched point pairs into correspondence vector
    Eigen::MatrixX3d src_matched_mat;
    Eigen::MatrixX3d tgt_matched_mat;
    pcl2eigen(src_matched_cloud, src_matched_mat);
    pcl2eigen(tgt_matched_cloud, tgt_matched_mat);

    for (int matched_index = 0; matched_index < src_matched_mat.rows(); matched_index++) {
        this->initial_correspondences.points.first.col(index) << src_matched_mat(matched_index, 0), src_matched_mat(
                matched_index, 1), src_matched_mat(matched_index, 2);
        this->initial_correspondences.points.second.col(index) << tgt_matched_mat(matched_index, 0), tgt_matched_mat(
                matched_index, 1), tgt_matched_mat(matched_index, 2);
        this->initial_correspondences.indices.first.push_back(matched_index);
        this->initial_correspondences.indices.second.push_back(matched_index);
        this->src_covariances_matched_.emplace_back(src_matched_covariances[matched_index]);
        this->tgt_covariances_matched_.emplace_back(src_matched_covariances[matched_index]);
        index++;
    }

    // feed paired points into solver
    this->matched_src_ = this->initial_correspondences.points.first;
    this->matched_tgt_ = this->initial_correspondences.points.second;

    this->solver.solve(this->initial_correspondences.points.first,
                       this->initial_correspondences.points.second,
                       this->src_covariances_matched_,
                       this->tgt_covariances_matched_);

    // extract solution
    auto solution = this->solver.getSolution();
    this->solution_ = Eigen::Matrix4d::Identity();
    this->solution_.block<3, 3>(0, 0) = solution.rotation;
    this->solution_.block<3, 1>(0, 3) = solution.translation;

    this->max_clique_ = this->solver.getInlierMaxClique();

}

void semanticTeaser::solve_for_multiclass_with_cov(const std::vector <pcl::PointCloud<pcl::PointXYZ>> &src_cloud_vec,
                                                   const std::vector <pcl::PointCloud<pcl::PointXYZ>> &tgt_cloud_vec,
                                                   const std::vector <Eigen::Matrix3d> &src_covariances,
                                                   const std::vector <Eigen::Matrix3d> &tgt_covariances) {
    int total_corres_size = 0;
    int src_node_size = 0;
    int tgt_node_size = 0;
    for (int i = 0; i < src_cloud_vec.size(); i++) {
        src_node_size += src_cloud_vec[i].points.size();
        tgt_node_size += tgt_cloud_vec[i].points.size();
        total_corres_size = total_corres_size + src_cloud_vec[i].points.size() * tgt_cloud_vec[i].points.size();
    }

//    std::cout << "\033[1;31mSize sum up: " << "src " << src_node_size << "   tgt " << tgt_node_size << "   corres " << total_corres_size << ".\033[0m" << std::endl;

    this->initial_correspondences.points = std::make_pair(Eigen::Matrix3Xd::Zero(3, total_corres_size),
                                                          Eigen::Matrix3Xd::Zero(3, total_corres_size));

    this->src_covariances_matched_.reserve(total_corres_size);
    this->tgt_covariances_matched_.reserve(total_corres_size);

    int class_num = src_cloud_vec.size();
    for (int indx = 0; indx < class_num; ++indx) {
        Eigen::MatrixX3d src_mat;
        Eigen::MatrixX3d tgt_mat;
        pcl2eigen(src_cloud_vec[indx], src_mat);
        pcl2eigen(tgt_cloud_vec[indx], tgt_mat);
        // auto sub_correspondences = culculate_correspondence(src_mat, tgt_mat);

        int index = 0;
        // build all to all correspondence
        for (int src_index = 0; src_index < src_mat.rows(); src_index++) {
            for (int tgt_index = 0; tgt_index < tgt_mat.rows(); tgt_index++) {
                this->initial_correspondences.points.first.col(index) << src_mat(src_index, 0), src_mat(src_index,
                                                                                                        1), src_mat(
                        src_index, 2);
                this->initial_correspondences.points.second.col(index) << tgt_mat(tgt_index, 0), tgt_mat(tgt_index,
                                                                                                         1), tgt_mat(
                        tgt_index, 2);
                this->initial_correspondences.indices.first.push_back(src_index);
                this->initial_correspondences.indices.second.push_back(tgt_index);
                this->src_covariances_matched_.emplace_back(src_covariances[src_index]);
                this->tgt_covariances_matched_.emplace_back(tgt_covariances[tgt_index]);
                // this->tgt_covariances_matched_[index] = tgt_covariances[tgt_index];
                index++;
            }
        }

    }

    this->matched_src_ = this->initial_correspondences.points.first;
    this->matched_tgt_ = this->initial_correspondences.points.second;

    this->solver.solve(this->initial_correspondences.points.first,
                       this->initial_correspondences.points.second,
                       this->src_covariances_matched_,
                       this->tgt_covariances_matched_);

    // extract solution
    auto solution = this->solver.getSolution();
    this->solution_ = Eigen::Matrix4d::Identity();
    this->solution_.block<3, 3>(0, 0) = solution.rotation;
    this->solution_.block<3, 1>(0, 3) = solution.translation;

    this->max_clique_ = this->solver.getInlierMaxClique();
}

void semanticTeaser::solve_for_multiclass_with_cov(const pcl::PointCloud <pcl::PointXYZ> &src_matched_cloud,
                                                   const pcl::PointCloud <pcl::PointXYZ> &tgt_matched_cloud,
                                                   const std::vector <Eigen::Matrix3d> &src_matched_covariances,
                                                   const std::vector <Eigen::Matrix3d> &tgt_matched_covariances) {

    // src_cloud_vec 包含多个类别的点云，每个类别有多个高斯分布
    int total_corres_size = 0;

    total_corres_size += src_matched_cloud.points.size();

    std::cout << "\033[1;31mcorres " << total_corres_size << ".\033[0m" << std::endl;

    this->initial_correspondences.points = std::make_pair(Eigen::Matrix3Xd::Zero(3, total_corres_size),
                                                          Eigen::Matrix3Xd::Zero(3, total_corres_size));

    this->src_covariances_matched_.reserve(total_corres_size);
    this->tgt_covariances_matched_.reserve(total_corres_size);

    int index = 0;// index through the whole matched vector

    // append matched point pairs into correspondence vector
    Eigen::MatrixX3d src_matched_mat;
    Eigen::MatrixX3d tgt_matched_mat;
    pcl2eigen(src_matched_cloud, src_matched_mat);
    pcl2eigen(tgt_matched_cloud, tgt_matched_mat);

    for (int matched_index = 0; matched_index < src_matched_mat.rows(); matched_index++) {
        this->initial_correspondences.points.first.col(index) << src_matched_mat(matched_index, 0), src_matched_mat(
                matched_index, 1), src_matched_mat(matched_index, 2);
        this->initial_correspondences.points.second.col(index) << tgt_matched_mat(matched_index, 0), tgt_matched_mat(
                matched_index, 1), tgt_matched_mat(matched_index, 2);
        this->initial_correspondences.indices.first.push_back(matched_index);
        this->initial_correspondences.indices.second.push_back(matched_index);
        this->src_covariances_matched_.emplace_back(src_matched_covariances[matched_index]);
        this->tgt_covariances_matched_.emplace_back(src_matched_covariances[matched_index]);
        index++;
    }

    // feed paired points into solver
    this->matched_src_ = this->initial_correspondences.points.first;
    this->matched_tgt_ = this->initial_correspondences.points.second;

    this->solver.solve(this->initial_correspondences.points.first,
                       this->initial_correspondences.points.second,
                       this->src_covariances_matched_,
                       this->tgt_covariances_matched_);

    // extract solution
    auto solution = this->solver.getSolution();
    this->solution_ = Eigen::Matrix4d::Identity();
    this->solution_.block<3, 3>(0, 0) = solution.rotation;
    this->solution_.block<3, 1>(0, 3) = solution.translation;

    this->max_clique_ = this->solver.getInlierMaxClique();
}

void semanticTeaser::culculate_correspondence(const Eigen::MatrixX3d &src,
                                              const Eigen::MatrixX3d &tgt) {
    // assert()
    int temp_size = src.rows() * tgt.rows();

    this->initial_correspondences.points.first.conservativeResize(Eigen::NoChange, temp_size);
    this->initial_correspondences.points.second.conservativeResize(Eigen::NoChange, temp_size);

    int index = 0;
    // build all to all correspondence
    for (int src_index = 0; src_index < src.rows(); src_index++) {
        for (int tgt_index = 0; tgt_index < tgt.rows(); tgt_index++) {
            this->initial_correspondences.points.first.col(index) << src(src_index, 0), src(src_index, 1), src(
                    src_index, 2);
            this->initial_correspondences.points.second.col(index) << tgt(tgt_index, 0), tgt(tgt_index, 1), tgt(
                    tgt_index, 2);
            this->initial_correspondences.indices.first.push_back(src_index);
            this->initial_correspondences.indices.second.push_back(tgt_index);
            index++;
        }
    }

}





