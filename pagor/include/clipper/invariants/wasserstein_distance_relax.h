//
// Created by qzj on 23-2-25.
//

#ifndef SRC_WASSERSTEIN_DISTANCE_RELAX_H
#define SRC_WASSERSTEIN_DISTANCE_RELAX_H

#include "abstract.h"

namespace clipper {


    /**
     * @brief      Specialization of PairwiseInvariant to Euclidean distance in
     *             the real numbers using the 2-norm as the invariant.
     */
    class WassersteinDistanceRelax : public PairwiseInvariant {
    public:
        struct Params {
            double sigma = 0.01; ///< spread / "variance" of exponential kernel
            std::vector<double> epsilon_vec; ///< bound on consistency score, determines if inlier/outlier 5.54*sigma with 99.999% confidence
            std::vector<double> cov_eps_vec;
            double mindist = 0; ///< minimum allowable distance between inlier points in the same dataset
            bool small_to_large = true;
        };
        int nb_num;
    public:
        WassersteinDistanceRelax(const Params &params)
                : params_(params) {
            nb_num = params_.epsilon_vec.size();
            // from large to small
            if (params_.small_to_large) {
                std::sort(params_.epsilon_vec.begin(), params_.epsilon_vec.end());
                std::sort(params_.cov_eps_vec.begin(), params_.cov_eps_vec.end());
            } else {
                std::sort(params_.epsilon_vec.begin(), params_.epsilon_vec.end(), std::greater<double>());
                std::sort(params_.cov_eps_vec.begin(), params_.cov_eps_vec.end(), std::greater<double>());
            }
        }

        ~WassersteinDistanceRelax() = default;

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
                        std::vector<double> &vec_c) override;

        int get_nb_num() {
            return nb_num;
        }

    private:
        Params params_;
    };

    using WassersteinDistanceRelaxPtr = std::shared_ptr<WassersteinDistanceRelax>;


} // ns clipper
#endif //SRC_WASSERSTEIN_DISTANCE_RELAX_H
