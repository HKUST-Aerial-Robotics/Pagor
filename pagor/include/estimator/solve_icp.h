//
// Created by qzj on 23-2-26.
//

#ifndef SRC_SOLVE_ICP_H
#define SRC_SOLVE_ICP_H

#include "point2pointFactor.h"
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/GncOptimizer.h>


namespace gtsam{
    using symbol_shorthand::X;  // pose

    inline void solve3d(const Eigen::Matrix3Xd& src, const Eigen::Matrix3Xd& tgt,
                        Eigen::Matrix4d& T, const Eigen::Matrix4d& T_init = Eigen::Matrix4d::Identity()){

        // Create a factor graph
        NonlinearFactorGraph graph;
        Values initial;
        initial.insert(X(0), Pose3(T_init));
        noiseModel::Diagonal::shared_ptr noise = noiseModel::Unit::Create(3);
        //auto huber = noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(1.345), noise);
        assert(src.cols() == tgt.cols());
        for (int i = 0; i < src.cols(); ++i) {
            graph.add(Point2pFactor(X(0), src.col(i), tgt.col(i), noise));
        }
        GncParams<LevenbergMarquardtParams> gncParams;
        auto gnc = GncOptimizer<GncParams<LevenbergMarquardtParams>>(graph,
                                                                     initial,
                                                                     gncParams);
        Values estimate = gnc.optimize();

        //GaussNewtonParams params;
        //params.setMaxIterations(10);
        //GaussNewtonOptimizer optimizer(graph, initial, params);
        //Values estimate = optimizer.optimize();

        T = estimate.at<Pose3>(X(0)).matrix();
    }
}

inline void solveBySVD(Eigen::MatrixXd &src, Eigen::MatrixXd &tgt, Eigen::Matrix4d &T) {
    // src: 3 x N, tgt: 3 x N
    Eigen::MatrixXd src_mean = src.rowwise().mean();
    Eigen::MatrixXd tgt_mean = tgt.rowwise().mean();
    Eigen::MatrixXd src_centered = src - src_mean.replicate(1, src.cols());
    Eigen::MatrixXd tgt_centered = tgt - tgt_mean.replicate(1, tgt.cols());
    Eigen::MatrixXd H = src_centered * tgt_centered.transpose();
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::MatrixXd U = svd.matrixU();
    Eigen::MatrixXd V = svd.matrixV();
    Eigen::MatrixXd R_ = V * U.transpose();
    if (R_.determinant() < 0) {
        V.col(2) *= -1;
        R_ = V * U.transpose();
    }
    T = Eigen::Matrix4d::Identity();
    T.block(0, 0, 3, 3) = R_;
    T.block(0, 3, 3, 1) = tgt_mean - R_ * src_mean;
}

#endif //SRC_SOLVE_ICP_H
