/**
 * @file clipper.cpp
 * @brief CLIPPER data association framework
 * @author Parker Lusk <plusk@mit.edu>
 * @date 19 March 2022
 */

#include <iostream>
#include <iomanip>
#include "clipper/clipper.h"
#include "clipper/utils.h"
#include <yaml-cpp/yaml.h>
#include <glog/logging.h>
#include "estimator/solve_icp.h"
#include "CommonFunc.h"

namespace clipper {

    CLIPPER::CLIPPER(const Params &params) : params_(params) {}

    CLIPPER::CLIPPER(const std::string &config_file) {

        YAML::Node config_node = YAML::LoadFile(config_file);

        YAML::Node metric_node = config_node["metric"];
        std::string metric_name = metric_node["name"].as<std::string>();
        // point euclidean distance
        if (metric_name == "EUCLID") {
            //double sigma = metric_node["sigma"].as<double>();
            //double epsilon = metric_node["epsilon"].as<double>();
            //const auto invariant = build_euclidean_distance_invariant(sigma, epsilon);
            //LOG(INFO) << "Add metric: " << metric_name << " with sigma: " << sigma << " and epsilon: " << epsilon << std::endl;
        } else if (metric_name == "EUCLID_RELAXED") {
            double sigma = metric_node["sigma"].as<double>();
            std::vector<double> epsilon_vec;
            for (const auto &epsilon: metric_node["noise_level_list"]) {
                epsilon_vec.push_back(epsilon.as<double>());
            }
            invariant_ = build_euclidean_distance_relax_invariant(sigma, epsilon_vec);
            //LOG(INFO) << "Add metric: " << metric_name << " with sigma: " << sigma << " and epsilon_vec";
            //for (const auto &epsilon : epsilon_vec) {
            //    LOG(INFO) << epsilon;
            //}
        } else if (metric_name == "WASSERSTEIN_RELAXED") {
            double sigma = metric_node["sigma"].as<double>();
            std::vector<double> epsilon_vec;
            for (const auto &epsilon: metric_node["noise_level_list"]) {
                epsilon_vec.push_back(epsilon.as<double>());
            }
            std::vector<double> cov_eps_vec;
            for (const auto &cov_eps: metric_node["cov_thd_list"]) {
                cov_eps_vec.push_back(cov_eps.as<double>());
            }
            invariant_ = build_wasserstein_distance_relax_invariant(sigma, epsilon_vec, cov_eps_vec, true);
        }

        params_.tol_u = config_node["tol_u"].as<double>();
        params_.tol_F = config_node["tol_F"].as<double>();
        params_.tol_Fop = config_node["tol_Fop"].as<double>();
        params_.maxiniters = config_node["maxiniters"].as<int>();
        params_.maxoliters = config_node["maxoliters"].as<int>();
        params_.beta = config_node["beta"].as<double>();
        params_.maxlsiters = config_node["maxlsiters"].as<int>();
        params_.eps = config_node["eps"].as<double>();
        params_.affinityeps = config_node["affinityeps"].as<double>();
    }

    // ----------------------------------------------------------------------------
    void CLIPPER::scorePairwiseConsistency(const Data &D1, const Data &D2,
                                           const Association &A) {
        //createAffinityMatrix, for each point in D1, construct a relationship with each point in D2, and then construct a matrix (D1.size()*D2.size()) x 2
        // A_: two columns, the first column is the index of the point in D1, and the second column is the index of the point in D2
        if (A.size() == 0) A_ = utils::createAllToAll(D1.cols(), D2.cols());
        else A_ = A;
        // get number of possible matches
        const size_t m = A_.rows();

        // There is need to add parallelization here, otherwise it will be slower
        std::vector <Eigen::MatrixXd> vM;
        vM.resize(invariant_->get_nb_num());
        for (size_t i = 0; i < invariant_->get_nb_num(); ++i) {
            vM[i] = Eigen::MatrixXd::Zero(m, m);
        }

#pragma omp parallel for shared(A_, D1, D2, vM_, vC_) if(parallelize_)
        // adjacency matrix M has m*m elements in total, and half of them without diagonal is m*(m-1)/2 elements, so the number of loops here is m*(m-1)/2
        for (size_t k = 0; k < m * (m - 1) / 2; ++k) {
            size_t i, j;// upper triangular matrix row and column number
            std::tie(i, j) = utils::k2ij(k, m); // k2ij function converts k to i and j, and the range of i and j is 0~m-1

            // if two matches contain the same point, then the consistency of this pair of matches is not calculated, because a point can only match one point
            if (A_(i, 0) == A_(j, 0) || A_(i, 1) == A_(j, 1)) {
                // violates distinctness constraint
                continue;
            }

            // Evaluate the consistency of geometric invariants associated with ei, ej
            // points to extract invariant from in D1
            const auto &d1i = D1.col(A_(i, 0));
            const auto &d1j = D1.col(A_(j, 0));
            //// points to extract invariant from in D2
            const auto &d2i = D2.col(A_(i, 1));
            const auto &d2j = D2.col(A_(j, 1));

            //计算二阶距离
            std::vector<double> vec_c;
            const bool succ = (*invariant_)(d1i, d1j, d2i, d2j, vec_c);
            // sparsity-promoting threshold for affinities
            for (size_t k = 0; k < vec_c.size(); ++k) {
                if (vec_c[k] > params_.affinityeps && succ) {
                    vM[k](i, j) = vec_c[k];
                }
            }
        }
        vM_.resize(invariant_->get_nb_num());
        vC_.resize(invariant_->get_nb_num());
        for (size_t i = 0; i < invariant_->get_nb_num(); ++i) {
            vM_[i] = vM[i].sparseView();
        }
        //C is a sparse matrix with two parts
        //nonzero entries: sorted by column, format (value, row), value is the value in M, and row is the row number in M
        //outer pointers: the first element of each column is the position in nonzero entries
        for (size_t i = 0; i < invariant_->get_nb_num(); ++i) {
            vC_[i] = vM_[i];
            //把非零元素都变成1
            vC_[i].coeffs() = 1;
        }
    }


    void CLIPPER::solve_for_multiclass_with_cov(const std::vector <pcl::PointCloud<pcl::PointXYZ>> &src_cloud_vec,
                                                const std::vector <pcl::PointCloud<pcl::PointXYZ>> &tgt_cloud_vec,
                                                const Covariances &src_covariances,
                                                const Covariances &tgt_covariances,
                                                const pcl::PointCloud <pcl::PointXYZ> &src_matched_cloud,
                                                const pcl::PointCloud <pcl::PointXYZ> &tgt_matched_cloud,
                                                const Covariances &src_matched_covariances,
                                                const Covariances &tgt_matched_covariances) {

        // src_cloud_vec contains point clouds of multiple categories, each category has multiple Gaussian distributions
        int total_corres_size = 0;
        int src_node_size = 0;
        int tgt_node_size = 0;
        for (int i = 0; i < src_cloud_vec.size(); i++) {
            //every instance is matched with all instances
            src_node_size += src_cloud_vec[i].points.size();
            tgt_node_size += tgt_cloud_vec[i].points.size();
            total_corres_size = total_corres_size + src_cloud_vec[i].points.size() * tgt_cloud_vec[i].points.size();
        }
        //LOG(INFO) << "src_node_size: " << src_node_size << " tgt_node_size: " << tgt_node_size << " total_corres_size: " << total_corres_size;
        //            << " src_fpfh: " << src_matched_cloud.points.size() << " tgt_fpfh: " << tgt_matched_cloud.points.size();
        total_corres_size += src_matched_cloud.points.size();

        A_ = Eigen::MatrixXi::Zero(total_corres_size, 2);
        src_raw_ = Eigen::Matrix3Xd::Zero(3, src_node_size + src_matched_cloud.points.size());
        tgt_raw_ = Eigen::Matrix3Xd::Zero(3, tgt_node_size + tgt_matched_cloud.points.size());
        Covariances src_covariances_matched;
        Covariances tgt_covariances_matched;
        //merge src_covariances and src_matched_covariances
        src_covariances_matched.insert(src_covariances_matched.end(), src_covariances.begin(), src_covariances.end());
        src_covariances_matched.insert(src_covariances_matched.end(), src_matched_covariances.begin(),
                                       src_matched_covariances.end());
        //merge tgt_covariances and tgt_matched_covariances
        tgt_covariances_matched.insert(tgt_covariances_matched.end(), tgt_covariances.begin(), tgt_covariances.end());
        tgt_covariances_matched.insert(tgt_covariances_matched.end(), tgt_matched_covariances.begin(),
                                       tgt_matched_covariances.end());
        //LOG(INFO) << "total_corres_size: " << total_corres_size << " src_node_size: " << src_node_size << " tgt_node_size: " << tgt_node_size
        //<< " src_matched_cloud: " << src_matched_cloud.points.size() << " tgt_matched_cloud: " << tgt_matched_cloud.points.size();
        //LOG(INFO) << "src_covariances size: " << src_covariances.size() << " tgt_covariances size: " << tgt_covariances.size()
        //<< " src_matched_covariances size: " << src_matched_covariances.size() << " tgt_matched_covariances size: " << tgt_matched_covariances.size();

        int index = 0;// index through the whole matched vector
        int class_num = src_cloud_vec.size();
        int src_index = 0;
        int tgt_index = 0;
        for (int class_indx = 0; class_indx < class_num; ++class_indx) {
            Eigen::MatrixX3d src_mat;
            Eigen::MatrixX3d tgt_mat;
            pcl2mat(src_cloud_vec[class_indx], src_mat);
            pcl2mat(tgt_cloud_vec[class_indx], tgt_mat);
            src_raw_.block(0, src_index, 3, src_mat.rows()) = src_mat.transpose();
            tgt_raw_.block(0, tgt_index, 3, tgt_mat.rows()) = tgt_mat.transpose();

            // build all to all correspondence
            for (int i = 0; i < src_mat.rows(); i++) {
                for (int j = 0; j < tgt_mat.rows(); j++) {
                    A_(index, 0) = i + src_index;
                    A_(index, 1) = j + tgt_index;
                    index++;
                }
            }
            src_index += src_mat.rows();
            tgt_index += tgt_mat.rows();
        }

        // append matched point pairs into correspondence vector
        Eigen::MatrixX3d src_matched_mat;
        Eigen::MatrixX3d tgt_matched_mat;
        pcl2mat(src_matched_cloud, src_matched_mat);
        pcl2mat(tgt_matched_cloud, tgt_matched_mat);
        src_raw_.block(0, src_index, 3, src_matched_mat.rows()) = src_matched_mat.transpose();
        tgt_raw_.block(0, tgt_index, 3, tgt_matched_mat.rows()) = tgt_matched_mat.transpose();
        for (int matched_index = 0; matched_index < src_matched_mat.rows(); matched_index++) {
            A_(index, 0) = src_node_size + matched_index;
            A_(index, 1) = tgt_node_size + matched_index;
            index++;
        }
#ifdef TEST_RUNTIME
        static double total_time = 0;
        static double total_count = 0;
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#endif
        //for (int i = 0; i < A_.rows(); ++i) {
        //LOG(INFO) << "src: " << src_raw_.col(A_(i, 0)).transpose() << " tgt: " << tgt_raw_.col(A_(i, 1)).transpose();
        //}

        scorePairwiseConsistencyGaussian(src_raw_, tgt_raw_, A_, src_covariances_matched, tgt_covariances_matched);
#ifdef TEST_RUNTIME
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        double time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
        total_time += time_used;
        total_count++;
        LOG(INFO) << "scorePairwiseConsistencyGaussian time: " << time_used << " avg time: " << total_time / total_count << " ms";
#endif
    }

    void CLIPPER::scorePairwiseConsistencyGaussian(const Data &D1, const Data &D2, const Association &A,
                                                   const Covariances &src_covariances_matched,
                                                   const Covariances &tgt_covariances_matched) {
        //createAffinityMatrix, for each point in D1, construct a relationship with each point in D2, and then construct a matrix (D1.size()*D2.size()) x 2
        // A_: two columns, the first column is the index of the point in D1, and the second column is the index of the point in D2
        if (A.size() == 0) A_ = utils::createAllToAll(D1.cols(), D2.cols());
        else A_ = A;
        //get the number of all possible matches
        const size_t m = A_.rows();

        // There is need to add parallelization here, otherwise it will be slower
        std::vector <Eigen::MatrixXd> vM;
        vM.resize(invariant_->get_nb_num());
        for (size_t i = 0; i < invariant_->get_nb_num(); ++i) {
            vM[i] = Eigen::MatrixXd::Zero(m, m);
        }

#pragma omp parallel for shared(A_, D1, D2, vM_, vC_) if(parallelize_)
        //adjacency matrix M has m*m elements in total, and half of them without diagonal is m*(m-1)/2 elements, so the number of cycles here is m*(m-1)/2
        for (size_t k = 0; k < m * (m - 1) / 2; ++k) {
            //upper triangular matrix row and column number
            size_t i, j;
            //k2ij function is to convert k into i and j, and the range of i and j is 0~m-1
            std::tie(i, j) = utils::k2ij(k, m);

            //if two pairs of matches contain the same point, then the consistency of this pair of matches is not calculated, because one point can only match one point
            if (A_(i, 0) == A_(j, 0) || A_(i, 1) == A_(j, 1)) {
                // violates distinctness constraint
                continue;
            }

            // Evaluate the consistency of geometric invariants associated with ei, ej
            // points to extract invariant from in D1
            const auto &d1i = D1.col(A_(i, 0));
            const auto &d1j = D1.col(A_(j, 0));
            //// points to extract invariant from in D2
            const auto &d2i = D2.col(A_(i, 1));
            const auto &d2j = D2.col(A_(j, 1));

            const auto &cov1i = src_covariances_matched[A_(i, 0)];
            const auto &cov1j = src_covariances_matched[A_(j, 0)];
            const auto &cov2i = tgt_covariances_matched[A_(i, 1)];
            const auto &cov2j = tgt_covariances_matched[A_(j, 1)];

            //compute the second-order distance
            std::vector<double> vec_c;
            const bool succ = (*invariant_)(d1i, d1j, d2i, d2j,
                                            cov1i, cov1j, cov2i, cov2j,
                                            vec_c);
            // sparsity-promoting threshold for affinities
            for (size_t k = 0; k < vec_c.size(); ++k) {
                if (vec_c[k] > params_.affinityeps && succ) {
                    vM[k](i, j) = vec_c[k];
                }
            }
        }
        vM_.resize(invariant_->get_nb_num());
        vC_.resize(invariant_->get_nb_num());
        for (size_t i = 0; i < invariant_->get_nb_num(); ++i) {
            //saveMatrixXdToTextFile(vM[i], "vM_" + std::to_string(i) + ".txt");
            vM_[i] = vM[i].sparseView();
        }
        //C is a sparse matrix with two parts
        //nonzero entries: sorted by column, format (value, row), value is the value in M, and row is the row number in M
        //outer pointers: the first element of each column is the position in nonzero entries
        for (size_t i = 0; i < invariant_->get_nb_num(); ++i) {
            vC_[i] = vM_[i];
            //把非零元素都变成1
            vC_[i].coeffs() = 1;
        }
    }

    // ----------------------------------------------------------------------------

    void CLIPPER::solve(const Eigen::VectorXd &_u0) {

#ifdef TEST_RUNTIME
        static double total_time_solve = 0;
        static double total_count_solve = 0;
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#endif
        Eigen::VectorXd u0;
        if (_u0.size() == 0) {
            // initialize u0 (A_.rows() x 1) to uniform distribution between 0 and 1
            u0 = utils::randvec(A_.rows());
            //u0 = Eigen::VectorXd::Ones(A_.rows());
        } else {
            u0 = _u0;
        }
        solns_.resize(invariant_->get_nb_num());
        findDenseClique(u0);
#ifdef TEST_RUNTIME
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        double time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
        total_time_solve += time_used;
        total_count_solve++;
        LOG(INFO) << "solve time: " << time_used << " avg time: " << total_time_solve / total_count_solve << " ms";
#endif
    }

    //    void CLIPPER::findMaxEigenvalue(Eigen::VectorXd &u, double d) {
    //        int n = A_.rows();
    //        Eigen::MatrixXd upper = M_.selfadjointView<Eigen::Upper>().matrix();
    //        Eigen::MatrixXd Md = upper.transpose() + upper + Eigen::MatrixXd::Identity(n, n);
    //        Md = (Md.array() > params_.eps).select(Md, -d);
    //        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(Md);
    //        Eigen::VectorXd v = es.eigenvectors().col(n - 1);
    //        double lambda = es.eigenvalues()(n - 1);
    //        if (lambda < 0) {
    //            lambda = 0;
    //            v = Eigen::VectorXd::Zero(n);
    //        }
    //        u = v.cwiseMax(0);
    //        u /= u.norm();
    //    }

    // ----------------------------------------------------------------------------

    Association CLIPPER::getInitialAssociations() {
        return A_;
    }

    // ----------------------------------------------------------------------------

    Associations CLIPPER::getSelectedAssociations() {
        Associations vA;
        vA.resize(invariant_->get_nb_num());
        for (int i = 0; i < vA.size(); ++i) {
            vA[i] = utils::selectInlierAssociations(solns_[i], A_);
        }
        return vA;
    }

    // ----------------------------------------------------------------------------

    //    Affinity CLIPPER::getAffinityMatrix() {
    //        Affinity M = SpAffinity(M_.selfadjointView<Eigen::Upper>())
    //                     + Affinity::Identity(M_.rows(), M_.cols());
    //        return M;
    //    }

    // ----------------------------------------------------------------------------

    //    Constraint CLIPPER::getConstraintMatrix() {
    //        Constraint C = SpConstraint(C_.selfadjointView<Eigen::Upper>())
    //                       + Constraint::Identity(C_.rows(), C_.cols());
    //        return C;
    //    }

    // ----------------------------------------------------------------------------

    //    void CLIPPER::setMatrixData(const Affinity &M, const Constraint &C) {
    //        Eigen::MatrixXd MM = M.triangularView<Eigen::Upper>();
    //        MM.diagonal().setZero();
    //        M_ = MM.sparseView();
    //
    //        Eigen::MatrixXd CC = C.triangularView<Eigen::Upper>();
    //        CC.diagonal().setZero();
    //        C_ = CC.sparseView();
    //    }

    // ----------------------------------------------------------------------------

    //    void CLIPPER::setSparseMatrixData(const SpAffinity &M, const SpConstraint &C) {
    //        M_ = M;
    //        C_ = C;
    //    }

    // ----------------------------------------------------------------------------
    // Private Methods
    // ----------------------------------------------------------------------------

    void CLIPPER::findDenseClique(const Eigen::VectorXd &u0) {
        const auto t1 = std::chrono::high_resolution_clock::now();

        //
        // Initialization
        //

        const size_t n = vM_[0].cols();
        const Eigen::VectorXd ones = Eigen::VectorXd::Ones(n);

        // initialize memory
        Eigen::VectorXd gradF(n);
        Eigen::VectorXd gradFnew(n);
        Eigen::VectorXd u(n);
        Eigen::VectorXd unew(n);
        Eigen::VectorXd Mu(n);
        Eigen::VectorXd num(n);
        Eigen::VectorXd den(n);

        // one step of power method to have a good scaling of u
        if (params_.rescale_u0) {
            u = vM_[0].selfadjointView<Eigen::Upper>() * u0 + u0;
        } else {
            u = u0;
        }
        u /= u.norm();

        // initial value of d
        double d = 0; // zero if there are no active constraints
        Eigen::VectorXd Cbu = ones * u.sum() - vC_[0].selfadjointView<Eigen::Upper>() * u - u;
        const Eigen::VectorXi idxD = ((Cbu.array() > params_.eps) && (u.array() > params_.eps)).cast<int>();
        if (idxD.sum() > 0) {
            Mu = vM_[0].selfadjointView<Eigen::Upper>() * u + u;
            num = utils::selectFromIndicator(Mu, idxD);
            den = utils::selectFromIndicator(Cbu, idxD);
            d = (num.array() / den.array()).mean();
        }

        // Orthogonal projected gradient ascent with homotopy

        double F = 0; // objective value

        size_t i, j, k; // iteration counters
        size_t relax = 0; // relaxation counter
        for (i = 0; i < params_.maxoliters; ++i) {

            const SpAffinity &M_ = vM_[relax];
            const SpConstraint &C_ = vC_[relax];

            gradF = (1 + d) * u - d * ones * u.sum() + M_.selfadjointView<Eigen::Upper>() * u +
                    C_.selfadjointView<Eigen::Upper>() * u * d;
            F = u.dot(gradF); // current objective value

            // Orthogonal projected gradient ascent
            for (j = 0; j < params_.maxiniters; ++j) {
                double alpha = 1;

                // Backtracking line search on gradient ascent
                double Fnew = 0, deltaF = 0;
                for (k = 0; k < params_.maxlsiters; ++k) {
                    unew = u + alpha * gradF;                     // gradient step
                    unew = unew.cwiseMax(0);                      // project onto positive orthant
                    unew.normalize();                             // project onto S^n
                    gradFnew = (1 + d) * unew // because M/C is missing identity on diagonal
                               - d * ones * unew.sum()
                               + M_.selfadjointView<Eigen::Upper>() * unew
                               + C_.selfadjointView<Eigen::Upper>() * unew * d;
                    Fnew = unew.dot(gradFnew);                    // new objective value after step

                    deltaF = Fnew - F;                            // change in objective value

                    if (deltaF < -params_.eps) {
                        // objective value decreased---we need to backtrack, so reduce step size
                        alpha = alpha * params_.beta;
                    } else {
                        break; // obj value increased, stop line search
                    }
                }
                const double deltau = (unew - u).norm();

                // update values
                F = Fnew;
                u = unew;
                gradF = gradFnew;

                // check if desired accuracy has been reached by gradient ascent
                if (deltau < params_.tol_u || std::abs(deltaF) < params_.tol_F) break;
            }

            //
            // Increase d
            //

            Cbu = ones * u.sum() - C_.selfadjointView<Eigen::Upper>() * u - u;
            const Eigen::VectorXi idxD = ((Cbu.array() > params_.eps) && (u.array() > params_.eps)).cast<int>();
            if (idxD.sum() > 0) {
                Mu = M_.selfadjointView<Eigen::Upper>() * u + u;
                num = utils::selectFromIndicator(Mu, idxD);
                den = utils::selectFromIndicator(Cbu, idxD);
                const double deltad = (num.array() / den.array()).abs().mean();

                d += deltad;

            } else {
                transform2Solution(u, F, i, t1, relax);
                relax++;
                if (relax == vM_.size())
                    break;
            }
        }
    }

    void CLIPPER::transform2Solution(const Eigen::VectorXd &u, double F, int i,
                                     const std::chrono::high_resolution_clock::time_point &t1,
                                     const size_t &relax) {

        // estimate cluster size using largest eigenvalue
        const int omega = std::round(F);

        // extract indices of nodes in identified dense cluster
        std::vector<int> I = utils::findIndicesOfkLargest(u, omega);

        const auto t2 = std::chrono::high_resolution_clock::now();
        const auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1);
        const double elapsed = static_cast<double>(duration.count()) / 1e9;

        // set solution
        Solution soln; ///< solution information from CLIPPER dense clique solver
        soln.t = elapsed;
        soln.ifinal = i;
        std::swap(soln.nodes, I);
        soln.u = u;
        soln.score = F;
        solns_[relax] = soln;
    }

    void
    CLIPPER::getMaxCliques(pcl::PointCloud <pcl::PointXYZ> &src_cloud, pcl::PointCloud <pcl::PointXYZ> &tgt_cloud) {

        const clipper::Associations vA = getSelectedAssociations();
        const clipper::Association Ain = vA[0];

        src_cloud.clear();
        src_cloud.reserve(Ain.rows());
        tgt_cloud.clear();
        tgt_cloud.reserve(Ain.rows());

        for (int idx = 0; idx < Ain.rows(); ++idx) {
            src_cloud.push_back(
                    pcl::PointXYZ(src_raw_(0, Ain(idx, 0)), src_raw_(1, Ain(idx, 0)), src_raw_(2, Ain(idx, 0))));
            tgt_cloud.push_back(
                    pcl::PointXYZ(tgt_raw_(0, Ain(idx, 1)), tgt_raw_(1, Ain(idx, 1)), tgt_raw_(2, Ain(idx, 1))));
        }
    }

    void CLIPPER::check(const Eigen::Matrix4d &pose) {
        const clipper::Associations vA = getSelectedAssociations();
        const clipper::Association Ain = vA[0];

        Eigen::Matrix3Xd corres_A = Eigen::Matrix3Xd::Zero(3, Ain.rows());
        Eigen::Matrix3Xd corres_B = Eigen::Matrix3Xd::Zero(3, Ain.rows());
        for (int i = 0; i < Ain.rows(); ++i) {
            corres_A.col(i) = src_raw_.col(Ain(i, 0));
            corres_B.col(i) = tgt_raw_.col(Ain(i, 1));
        }
        //    compute the residuals
        Eigen::Matrix3Xd corres_A_transformed =
                pose.block(0, 0, 3, 3) * corres_A + pose.block(0, 3, 3, 1) * Eigen::MatrixXd::Ones(1, corres_B.cols());
        Eigen::Matrix3Xd residuals = corres_B - corres_A_transformed;
        LOG(INFO) << "check residuals: " << residuals.norm();
        for (int i = 0; i < residuals.cols(); ++i) {
            LOG(INFO) << "residuals: " << residuals.col(i).norm();
        }
    }

    void CLIPPER::getTransformationMatrix(CertifiedTransformations &transformations) {
        const clipper::Associations vA = getSelectedAssociations();
        transformations.resize(vA.size());
        for (int candi = 0; candi < vA.size(); ++candi) {
            const clipper::Association Ain = vA[candi];
            if (Ain.rows() < 3) {
                //LOG(WARNING) << "Too few correspondences to estimate a transformation.";
                transformations[candi].first = Eigen::Matrix4d::Identity();
                transformations[candi].second = false;
                continue;
            }
            Eigen::Matrix3Xd src = Eigen::Matrix3Xd::Zero(3, Ain.rows());
            Eigen::Matrix3Xd tgt = Eigen::Matrix3Xd::Zero(3, Ain.rows());
            //LOG(INFO) << "Ain.rows(): " << Ain.rows();
            for (int i = 0; i < Ain.rows(); ++i) {
                src.col(i) = src_raw_.col(Ain(i, 0));
                tgt.col(i) = tgt_raw_.col(Ain(i, 1));
                //LOG(INFO) << "Ain(i): " << Ain(i, 0) << ", " << Ain(i, 1) << ", src: " << src.col(i).transpose() << ", tgt: " << tgt.col(i).transpose();
            }

            Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
            gtsam::solve3d(src, tgt, T);
            //solveBySVD(src, tgt, T);
            transformations[candi].first = T;
            transformations[candi].second = true;
        }
        //for (auto &transformation : transformations) {
        //    LOG(INFO) << "transformation: \n" << transformation.first;
        //}
    }

    Eigen::Matrix4d
    CLIPPER::getTransformationMatrix(const Eigen::Matrix3Xd &D1, const Eigen::Matrix3Xd &D2, const Association &A) {
        Eigen::Matrix3Xd corres_A = Eigen::Matrix3Xd::Zero(3, A.rows());
        Eigen::Matrix3Xd corres_B = Eigen::Matrix3Xd::Zero(3, A.rows());
        for (int i = 0; i < A.rows(); ++i) {
            corres_A.col(i) = D1.col(A(i, 0));
            corres_B.col(i) = D2.col(A(i, 1));
        }
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        Eigen::Matrix3d R;
        Eigen::Vector3d t;
        Eigen::Vector3d c1 = corres_A.rowwise().mean();
        Eigen::Vector3d c2 = corres_B.rowwise().mean();
        corres_A.colwise() -= c1;
        corres_B.colwise() -= c2;
        Eigen::Matrix3d H = corres_A * corres_B.transpose();
        Eigen::JacobiSVD <Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
        R = svd.matrixU() * svd.matrixV().transpose();
        if (R.determinant() < 0) {
            R.col(2) *= -1;
        }
        t = c2 - R * c1;
        T.block(0, 0, 3, 3) = R;
        T.block(0, 3, 3, 1) = t;
        return T;
    }
} // ns clipper