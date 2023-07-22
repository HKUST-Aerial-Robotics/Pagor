/**
 * @file euclidean_distance.h
 * @brief Pairwise Euclidean distance geometric invariant
 * @author Parker Lusk <plusk@mit.edu>
 * @date 15 May 2021
 */

#pragma once

#include "abstract.h"

namespace clipper {


    /**
     * @brief      Specialization of PairwiseInvariant to Euclidean distance in
     *             the real numbers using the 2-norm as the invariant.
     */
    class EuclideanDistance : public PairwiseInvariant {
    public:
        struct Params {
            double sigma = 0.01; ///< spread / "variance" of exponential kernel
            double epsilon = 0.06; ///< bound on consistency score, determines if inlier/outlier 5.54*sigma with 99.999% confidence
            double mindist = 0; ///< minimum allowable distance between inlier points in the same dataset
        };
    public:
        EuclideanDistance(const Params &params)
                : params_(params) {}

        ~EuclideanDistance() = default;

        /**
         * @brief      Functor for pairwise invariant scoring function
         *
         * @param[in]  ai    Element i from dataset 1
         * @param[in]  aj    Element j from dataset 1
         * @param[in]  bi    Element i from dataset 2
         * @param[in]  bj    Element j from dataset 2
         *
         * @return     The consistency score for the association of (ai,bi) and (aj,bj)
         */
        double operator()(const Datum &ai, const Datum &aj, const Datum &bi, const Datum &bj) override;

        bool
        operator()(const Datum &ai, const Datum &aj, const Datum &bi, const Datum &bj, std::vector<double> &vec_c) {
            return false;
        }

        int get_nb_num() {
            return 0;
        }

    private:
        Params params_;
    };

    using EuclideanDistancePtr = std::shared_ptr<EuclideanDistance>;


} // ns clipper