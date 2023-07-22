//
// Created by qzj on 23-2-25.
//

#include "clipper/invariants/wasserstein_distance_relax.h"
#include <unsupported/Eigen/MatrixFunctions>
#include "glog/logging.h"

namespace clipper {

    bool WassersteinDistanceRelax::operator()(const Datum &ai, const Datum &aj, const Datum &bi, const Datum &bj,
                                              const Covariance &cov_ai, const Covariance &cov_aj,
                                              const Covariance &cov_bi, const Covariance &cov_bj,
                                              std::vector<double> &vec_c) {


        // distance between two points in the same cloud
        const double l1 = (ai - aj).norm();
        const double l2 = (bi - bj).norm();

        Covariance cov_l1 = cov_ai + cov_aj;
        Covariance cov_l2 = cov_bi + cov_bj;
        // enforce minimum distance criterion -- if points in the same dataset
        // are too close, then this pair of associations cannot be selected
        if (params_.mindist > 0 && (l1 < params_.mindist || l2 < params_.mindist)) {
            return false;
        }

        // consistency score
        //const double c = std::abs(l1 - l2);
        const double s1 = abs(l1 / l2 - 1);
        const double s2 = abs(l2 / l1 - 1);

        //LOG(INFO) << "s1: " << s1 << " s2: " << s2 << " abs(l1-l2): " << abs(l1-l2);
        vec_c.resize(nb_num);
        for (int i = 0; i < nb_num; ++i) {
            double prob;
            double beta = 2 * params_.epsilon_vec[i];
            if (s1 < beta && s2 < beta) {
                double s = std::min(s1, s2);
                //prob = std::exp(-0.5 * (s1 * s1 + s2 * s2) / (params_.sigma * params_.sigma));
                prob = std::exp(-0.5 * (s * s) / (params_.sigma * params_.sigma));
                //prob = 1;
                //const double w_dist = (cov_l1 + cov_l2 - 2 * (cov_l1.sqrt() * cov_l2 * cov_l1.sqrt()).sqrt()).trace();
                //if (w_dist < params_.cov_eps_vec[i]) {
                //    //prob = 1;
                //    prob = std::exp(-0.5 * (s1 * s1 + s2 * s2) / (params_.sigma * params_.sigma));
                //    LOG(INFO) << "w_dist: " << w_dist << " prob: " << prob;
                //} else {
                //    prob = 0;
                //}
            } else {
                prob = 0;
            }
            vec_c[i] = prob;
        }

        return true;
    }

    bool WassersteinDistanceRelax::operator()(const Datum &ai, const Datum &aj, const Datum &bi, const Datum &bj,
                                              std::vector<double> &vec_c) {


        // distance between two points in the same cloud
        const double l1 = (ai - aj).norm();
        const double l2 = (bi - bj).norm();

        // enforce minimum distance criterion -- if points in the same dataset
        // are too close, then this pair of associations cannot be selected
        if (params_.mindist > 0 && (l1 < params_.mindist || l2 < params_.mindist)) {
            return false;
        }

        // consistency score
        const double c = std::abs(l1 - l2);

        vec_c.resize(nb_num);
        for (int i = 0; i < nb_num; ++i) {
            double prob = 0.0;
            if (c < params_.epsilon_vec[i]) {
                prob = std::exp(-0.5 * c * c / (params_.sigma * params_.sigma));
            } else {
                prob = 0;
            }
            vec_c[i] = prob;
        }

        return true;
    }

    double WassersteinDistanceRelax::operator()(const Datum &ai, const Datum &aj,
                                                const Datum &bi, const Datum &bj) {

        // distance between two points in the same cloud
        const double l1 = (ai - aj).norm();
        const double l2 = (bi - bj).norm();

        // enforce minimum distance criterion -- if points in the same dataset
        // are too close, then this pair of associations cannot be selected
        if (params_.mindist > 0 && (l1 < params_.mindist || l2 < params_.mindist)) {
            return 0.0;
        }

        // consistency score
        const double c = std::abs(l1 - l2);

        return (c < params_.epsilon_vec[0]) ? std::exp(-0.5 * c * c / (params_.sigma * params_.sigma)) : 0;
    }
}