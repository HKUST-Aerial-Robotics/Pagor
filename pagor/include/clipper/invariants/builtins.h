/**
 * @file builtins.h
 * @brief Common geometric invariants for CLIPPER
 * @author Parker Lusk <plusk@mit.edu>
 * @date 3 October 2020
 */

#pragma once

#include "clipper/types.h"
#include "euclidean_distance.h"
#include "euclidean_distance_relax.h"
#include "clipper/invariants/wasserstein_distance_relax.h"

namespace clipper {

    PairwiseInvariantPtr build_euclidean_distance_relax_invariant(double sigma, std::vector<double> epsilon_vec);

    PairwiseInvariantPtr build_wasserstein_distance_relax_invariant(double sigma, std::vector<double> epsilon_vec,
                                                                    std::vector<double> cov_eps_vec,
                                                                    bool small_to_large);

} // ns clipper