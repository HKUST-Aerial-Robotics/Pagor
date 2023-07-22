//
// Created by qzj on 23-2-24.
//

#include "clipper/invariants/euclidean_distance_relax.h"

namespace clipper {

    bool EuclideanDistanceRelax::operator()(const Datum &ai, const Datum &aj, const Datum &bi, const Datum &bj,
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

    double EuclideanDistanceRelax::operator()(const Datum &ai, const Datum &aj,
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

        return (c < params_.epsilon) ? std::exp(-0.5 * c * c / (params_.sigma * params_.sigma)) : 0;
    }


} // ns clipper