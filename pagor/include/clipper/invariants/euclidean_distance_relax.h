//
// Created by qzj on 23-2-24.
//

#ifndef SRC_EUCLIDEAN_DISTANCE_RELAX_H
#define SRC_EUCLIDEAN_DISTANCE_RELAX_H

#include "abstract.h"

namespace clipper {


    /**
     * @brief      Specialization of PairwiseInvariant to Euclidean distance in
     *             the real numbers using the 2-norm as the invariant.
     */
    class EuclideanDistanceRelax : public PairwiseInvariant {
    public:
        struct Params {
            double sigma = 0.01; ///< spread / "variance" of exponential kernel
            double epsilon = 0.01; ///< bound on consistency score, determines if inlier/outlier 5.54*sigma with 99.999% confidence
            std::vector<double> epsilon_vec; ///< bound on consistency score, determines if inlier/outlier 5.54*sigma with 99.999% confidence
            double mindist = 0; ///< minimum allowable distance between inlier points in the same dataset
        };
        int nb_num;
    public:
        EuclideanDistanceRelax(const Params &params)
                : params_(params) {
            nb_num = params_.epsilon_vec.size();
            // from large to small
            std::sort(params_.epsilon_vec.begin(), params_.epsilon_vec.end(), std::greater<double>());
        }

        ~EuclideanDistanceRelax() = default;

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

        bool operator()(const Datum &ai, const Datum &aj, const Datum &bi, const Datum &bj,
                        std::vector<double> &vec_c) override;

        bool operator()(const Datum &ai, const Datum &aj, const Datum &bi, const Datum &bj,
                        const Covariance &cov_ai, const Covariance &cov_aj, const Covariance &cov_bi,
                        const Covariance &cov_bj,
                        std::vector<double> &vec_c) {
            return false;
        }

        int get_nb_num() {
            return nb_num;
        }

    private:
        Params params_;
    };

    using EuclideanDistanceRelaxPtr = std::shared_ptr<EuclideanDistanceRelax>;


} // ns clipper

#endif //SRC_EUCLIDEAN_DISTANCE_RELAX_H
