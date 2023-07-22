#ifndef SEMANTIC_TEASER_H
#define SEMANTIC_TEASER_H

#include "teaser/registration.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "conversion.hpp"

// abstract solver class
class AbstrctSolver {
protected:
    Eigen::Matrix4d solution_;

public:
    AbstrctSolver() : solution_(Eigen::Matrix4d::Identity()) {}

    // virtual ~AbstrctSolver() {}

    virtual void solve(const Eigen::MatrixX3d &src, const Eigen::MatrixX3d &tgt) = 0;

    const Eigen::Matrix4d &get_solution() { return this->solution_; }
};


class semanticTeaser : public AbstrctSolver {

public:
    Eigen::Matrix<double, 3, Eigen::Dynamic> matched_src_;
    Eigen::Matrix<double, 3, Eigen::Dynamic> matched_tgt_;

    using TEASER = teaser::RobustRegistrationSolver;

    struct Params {

        TEASER::Params teaser_params;

        Params() {
            teaser_params.reg_name = "TEASER";
            teaser_params.cote_mode = "weighted_mean";
            teaser_params.using_rot_inliers_when_estimating_cote = false;
            teaser_params.noise_bound = 0.06;
            teaser_params.distribution_noise_bound = 2.0;
            teaser_params.cbar2 = 1;
            teaser_params.estimate_scaling = false;
            teaser_params.rotation_gnc_factor = 1.4;
            teaser_params.rotation_estimation_algorithm = TEASER::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
            teaser_params.rotation_max_iterations = 100;
            teaser_params.rotation_cost_threshold = 1e-6;
            teaser_params.kcore_heuristic_threshold = 1;
            teaser_params.use_max_clique = true;
            teaser_params.max_clique_exact_solution = true;
            teaser_params.max_clique_time_limit = 3600;
            teaser_params.inlier_selection_mode = TEASER::INLIER_SELECTION_MODE::PMC_EXACT;
        }

    };


    struct Correspondences {

        std::pair<Eigen::Matrix3Xd, Eigen::Matrix3Xd> points;
        std::pair<std::vector<int>, std::vector<int>> indices;

        Correspondences() {
            indices.first.reserve(1000);
            indices.second.reserve(1000);
        }
    };

protected:

    semanticTeaser::Params params;

    TEASER solver;

    Correspondences initial_correspondences;

    std::vector<Eigen::Matrix3d> src_covariances_matched_;

    std::vector<Eigen::Matrix3d> tgt_covariances_matched_;

    // Max clique vector
    std::vector<int> max_clique_;

public:

    semanticTeaser(semanticTeaser::Params params) : params(params), solver(params.teaser_params) {
        // solver.reset(new TEASER(const params.teaser_params));
    }

    // culculate all to all correspondence
    void culculate_correspondence(const Eigen::MatrixX3d &src,
                                  const Eigen::MatrixX3d &tgt);

    void solve(const Eigen::MatrixX3d &src, const Eigen::MatrixX3d &tgt) override;

    /** \brief solve cloud-wise transformation using multiple semantic clusters
     * \param[in] src_cloud_vec vector containing source clouds of different semantics
     * \param[in] tgt_cloud_vec vector containing target clouds of different semantics
     */
    void solve_for_multiclass(const std::vector<pcl::PointCloud<pcl::PointXYZ>> &src_cloud_vec,
                              const std::vector<pcl::PointCloud<pcl::PointXYZ>> &tgt_cloud_vec);

    /** \brief solve cloud-wise transformation using multiple semantic clusters with cov mat outlier pruning
     * \param[in] src_cloud_vec vector containing source clouds of different semantics
     * \param[in] tgt_cloud_vec vector containing target clouds of different semantics
     * \param[in] src_covariances vector of eigen matrix containing cov mats of each point in src cloud
     * \param[in] tgt_covariances vector of eigen matrix containing cov mats of each point in tgt cloud
     */
    void solve_for_multiclass_with_cov(const std::vector<pcl::PointCloud<pcl::PointXYZ>> &src_cloud_vec,
                                       const std::vector<pcl::PointCloud<pcl::PointXYZ>> &tgt_cloud_vec,
                                       const std::vector<Eigen::Matrix3d> &src_covariances,
                                       const std::vector<Eigen::Matrix3d> &tgt_covariances);

    void solve_for_multiclass_with_cov(const pcl::PointCloud<pcl::PointXYZ> &src_matched_cloud,
                                       const pcl::PointCloud<pcl::PointXYZ> &tgt_matched_cloud,
                                       const std::vector<Eigen::Matrix3d> &src_matched_covariances,
                                       const std::vector<Eigen::Matrix3d> &tgt_matched_covariances);

    void solve_for_multiclass_with_cov(const std::vector<pcl::PointCloud<pcl::PointXYZ>> &src_cloud_vec,
                                       const std::vector<pcl::PointCloud<pcl::PointXYZ>> &tgt_cloud_vec,
                                       const std::vector<Eigen::Matrix3d> &src_covariances,
                                       const std::vector<Eigen::Matrix3d> &tgt_covariances,
                                       const pcl::PointCloud<pcl::PointXYZ> &src_matched_cloud,
                                       const pcl::PointCloud<pcl::PointXYZ> &tgt_matched_cloud,
                                       const std::vector<Eigen::Matrix3d> &src_matched_covariances,
                                       const std::vector<Eigen::Matrix3d> &tgt_matched_covariances);

    void setInliers(
            const Eigen::Matrix<double, 3, Eigen::Dynamic> &raw, pcl::PointCloud<pcl::PointXYZ> &inliers,
            const std::vector<int> &idx_inliers) {
        inliers.clear();
        inliers.reserve(idx_inliers.size());

        for (const int idx: idx_inliers) {
            inliers.push_back(pcl::PointXYZ(raw(0, idx), raw(1, idx), raw(2, idx)));
        }
    }

    void getMaxCliques(pcl::PointCloud<pcl::PointXYZ> &source_max_clique,
                       pcl::PointCloud<pcl::PointXYZ> &target_max_clique) {
        setInliers(matched_src_, source_max_clique, max_clique_);
        setInliers(matched_tgt_, target_max_clique, max_clique_);
    }

    void getInitCorr(pcl::PointCloud<pcl::PointXYZ> &source_matched,
                     pcl::PointCloud<pcl::PointXYZ> &target_matched) {
        eigen2pcl(matched_src_, source_matched);
        eigen2pcl(matched_tgt_, target_matched);
    }

};

#endif