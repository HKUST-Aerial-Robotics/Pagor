//
// Created by qzj on 19/12/2022.
//
#include "clipper/invariants/builtins.h"

namespace clipper {

    PairwiseInvariantPtr build_euclidean_distance_relax_invariant(double sigma, std::vector<double> epsilon_vec) {
        // instantiate the invariant function that will be used to score associations
        clipper::EuclideanDistanceRelax::Params iparams;
        iparams.sigma = sigma;
        iparams.epsilon_vec = epsilon_vec;
        clipper::EuclideanDistanceRelaxPtr invariant;
        invariant.reset(new clipper::EuclideanDistanceRelax(iparams));
        return invariant;
    }

    PairwiseInvariantPtr build_wasserstein_distance_relax_invariant(double sigma, std::vector<double> epsilon_vec,
                                                                    std::vector<double> cov_eps_vec,
                                                                    bool small_to_large) {
        // instantiate the invariant function that will be used to score associations
        clipper::WassersteinDistanceRelax::Params iparams;
        iparams.sigma = sigma;
        iparams.epsilon_vec = epsilon_vec;
        iparams.cov_eps_vec = cov_eps_vec;
        iparams.small_to_large = small_to_large;
        clipper::WassersteinDistanceRelaxPtr invariant;
        invariant.reset(new clipper::WassersteinDistanceRelax(iparams));
        return invariant;
    }
}