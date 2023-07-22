/**
 ============================================================================
 Name        : Parallel Maximum Clique (PMC) Library
 Author      : Ryan A. Rossi   (rrossi@purdue.edu)
 Description : A general high-performance parallel framework for computing
               maximum cliques. The library is designed to be fast for large
               sparse graphs.

 Copyright (C) 2012-2013, Ryan A. Rossi, All rights reserved.

 Please cite the following paper if used:
   Ryan A. Rossi, David F. Gleich, Assefaw H. Gebremedhin, Md. Mostofa
   Patwary, A Fast Parallel Maximum Clique Algorithm for Large Sparse Graphs
   and Temporal Strong Components, arXiv preprint 1302.6256, 2013.

 See http://ryanrossi.com/pmc for more information.
 ============================================================================
 */

#include "pmc/pmcx_maxclique.h"

using namespace std;
using namespace pmc;

int pmcx_maxclique::search(pmc_graph &G, vector<int> &sol) {

    vertices = G.get_vertices();
    edges = G.get_edges();
//    degree = G.get_degree();
    int *pruned = new int[G.num_vertices()];
    memset(pruned, 0, G.num_vertices() * sizeof(int));
    int mc = lb, i = 0, u = 0;

    // initial pruning
    int lb_idx = G.initial_pruning(G, pruned, lb);

    // set to worst case bound of cores/coloring
    vector <Vertex> P, T;
    P.reserve(G.get_max_degree() + 1);
    T.reserve(G.get_max_degree() + 1);

    vector<int> C, C_max;
    C.reserve(G.get_max_degree() + 1);
    C_max.reserve(G.get_max_degree() + 1);

    // init the neigh coloring array
    // in theory, this should be G.get_max_core()+1, but
    // the starting color is 1, not 0, so that's +2
    // and then there is an extra off-by-one error that makes +3
    vector <vector<int>> colors(G.get_max_core() + 3);
    for (int i = 0; i < G.get_max_core() + 1; i++) colors[i].reserve(G.get_max_core() + 1);

    // order verts for our search routine
    vector <Vertex> V;
    V.reserve(G.num_vertices());
    G.order_vertices(V, G, lb_idx, lb, vertex_ordering, decr_order);
    DEBUG_PRINTF("|V| = %i\n", V.size());

    vector<short> ind(G.num_vertices(), 0);
    vector<int> es = G.get_edges_array();
    vector<long long> vs = G.get_vertices_array();

    vector<double> induce_time(num_threads, get_time());
    for (int t = 0; t < num_threads; ++t) induce_time[t] = induce_time[t] + t / 4;

#pragma omp parallel for schedule(dynamic) shared(pruned, G, T, V, mc, C_max, induce_time) \
        firstprivate(colors, ind, vs, es) private(u, P, C) num_threads(num_threads)
    for (i = 0; i < (V.size()) - (mc - 1); ++i) {
        if (not_reached_ub) {
            if (G.time_left(C_max, sec, time_limit, time_expired_msg)) {

                u = V[i].get_id();
                if ((*bound)[u] > mc) {
                    P.push_back(V[i]);
                    for (long long j = vs[u]; j < vs[u + 1]; ++j)
                        if (!pruned[es[j]])
                            if ((*bound)[es[j]] > mc)
                                P.push_back(Vertex(es[j], (vs[es[j] + 1] - vs[es[j]]))); /// local


                    if (P.size() > mc) {
                        neigh_cores_bound(vs, es, P, ind, mc);
                        if (P.size() > mc && P[0].get_bound() >= mc) {
                            neigh_coloring_bound(vs, es, P, ind, C, C_max, colors, pruned, mc);
                            if (P.back().get_bound() > mc) {
                                branch(vs, es, P, ind, C, C_max, colors, pruned, mc);
                            }
                        }
                    }
                    P = T;
                }
                pruned[u] = 1;

                // dynamically reduce graph in a thread-safe manner
                if ((get_time() - induce_time[omp_get_thread_num()]) > wait_time) {
                    G.reduce_graph(vs, es, pruned, G, i + lb_idx, mc);
                    G.graph_stats(G, mc, i + lb_idx, sec);
                    induce_time[omp_get_thread_num()] = get_time();
                }
            }
        }
    }
    if (pruned) delete[] pruned;

    sol.resize(mc);
    for (int i = 0; i < C_max.size(); i++) sol[i] = C_max[i];
    G.print_break();
    return sol.size();
}

void pmcx_maxclique::branch(
        vector<long long> &vs,
        vector<int> &es,
        vector <Vertex> &P,
        vector<short> &ind,
        vector<int> &C,
        vector<int> &C_max,
        vector <vector<int>> &colors,
        int *&pruned,
        int &mc) {

    // stop early if ub is reached
    if (not_reached_ub) {
        while (P.size() > 0) {
            // terminating condition
            if (C.size() + P.back().get_bound() > mc) {
                int v = P.back().get_id();
                C.push_back(v);

                vector <Vertex> R;
                R.reserve(P.size());
                for (long long j = vs[v]; j < vs[v + 1]; j++) ind[es[j]] = 1;

                // intersection of N(v) and P - {v}
                for (int k = 0; k < P.size() - 1; k++)
                    if (ind[P[k].get_id()])
                        if (!pruned[P[k].get_id()])
                            if ((*bound)[P[k].get_id()] > mc)
                                R.push_back(P[k]);

                for (long long j = vs[v]; j < vs[v + 1]; j++) ind[es[j]] = 0;


                if (R.size() > 0) {
                    // color graph induced by R and sort for O(1)
                    neigh_coloring_bound(vs, es, R, ind, C, C_max, colors, pruned, mc);
                    branch(vs, es, R, ind, C, C_max, colors, pruned, mc);
                } else if (C.size() > mc) {
                    // obtain lock
#pragma omp critical (update_mc)
                    if (C.size() > mc) {
                        // ensure updated max is flushed
                        mc = C.size();
                        C_max = C;
                        print_mc_info(C, sec);
                        if (mc >= param_ub) {
                            not_reached_ub = false;
                            DEBUG_PRINTF("[pmc: upper bound reached]  omega = %i\n", mc);
                        }
                    }
                }
                // backtrack and search another branch
                R.clear();
                C.pop_back();
            } else return;
            P.pop_back();
        }
    }
}

/**
 * Dense graphs: we use ADJ matrix + CSC Representation
 * ADJ:	  O(1) edge lookups
 * CSC:	  O(1) time to compute degree
 *
 */
int pmcx_maxclique::search_dense(pmc_graph &G, vector<int> &sol) {

    vertices = G.get_vertices();
    edges = G.get_edges();
//    degree = G.get_degree();
    auto adj = G.adj;

    int *pruned = new int[G.num_vertices()];
    memset(pruned, 0, G.num_vertices() * sizeof(int));
    int mc = lb, i = 0, u = 0;

    // initial pruning
    int lb_idx = G.initial_pruning(G, pruned, lb, adj);

    // set to worst case bound of cores/coloring
    vector <Vertex> P, T;
    P.reserve(G.get_max_degree() + 1);
    T.reserve(G.get_max_degree() + 1);

    vector<int> C, C_max;
    C.reserve(G.get_max_degree() + 1);
    C_max.reserve(G.get_max_degree() + 1);

    // init the neigh coloring array
    // in theory, this should be G.get_max_core()+1, but
    // the starting color is 1, not 0, so that's +2
    // and then there is an extra off-by-one error that makes +3
    vector <vector<int>> colors(G.get_max_core() + 3);
    for (int i = 0; i < G.get_max_core() + 1; i++) colors[i].reserve(G.get_max_core() + 1);

    // order verts for our search routine
    vector <Vertex> V;
    V.reserve(G.num_vertices());
    G.order_vertices(V, G, lb_idx, lb, vertex_ordering, decr_order);
    DEBUG_PRINTF("|V| = %i\n", V.size());

    vector<short> ind(G.num_vertices(), 0);
    vector<int> es = G.get_edges_array();
    vector<long long> vs = G.get_vertices_array();

    vector<double> induce_time(num_threads, get_time());
    for (int t = 0; t < num_threads; ++t) induce_time[t] = induce_time[t] + t / 4;


#pragma omp parallel for schedule(dynamic) shared(pruned, G, adj, T, V, mc, C_max, induce_time) \
        firstprivate(colors, ind, vs, es) private(u, P, C) num_threads(num_threads)
    for (i = 0; i < (V.size()) - (mc - 1); ++i) {
        DEBUG_PRINTF("DEBUG current mc: %i\n", mc);
        if (not_reached_ub) {
            if (G.time_left(C_max, sec, time_limit, time_expired_msg)) {

                u = V[i].get_id();
                if ((*bound)[u] > mc) {
                    P.push_back(V[i]);
                    for (long long j = vs[u]; j < vs[u + 1]; ++j)
                        if (!pruned[es[j]])
                            if ((*bound)[es[j]] > mc)
                                P.push_back(Vertex(es[j], (vs[es[j] + 1] - vs[es[j]]))); /// local

                    if (P.size() > mc) {
                        // neighborhood core ordering and pruning
                        neigh_cores_bound(vs, es, P, ind, mc);
                        if (P.size() > mc && P[0].get_bound() >= mc) {
                            neigh_coloring_dense(vs, es, P, ind, C, C_max, colors, mc, adj);
                            if (P.back().get_bound() > mc) {
                                branch_dense(vs, es, P, ind, C, C_max, colors, pruned, mc, adj);
                            }
                        }
                    }
                    P = T;
                }
                pruned[u] = 1;
                for (long long j = vs[u]; j < vs[u + 1]; j++) {
                    adj[u][es[j]] = false;
                    adj[es[j]][u] = false;
                }

                // dynamically reduce graph in a thread-safe manner
                if ((get_time() - induce_time[omp_get_thread_num()]) > wait_time) {
                    G.reduce_graph(vs, es, pruned, G, i + lb_idx, mc);
                    G.graph_stats(G, mc, i + lb_idx, sec);
                    induce_time[omp_get_thread_num()] = get_time();
                }
            }
        }
    }

    if (pruned) delete[] pruned;

    sol.resize(mc);
    for (int i = 0; i < C_max.size(); i++) sol[i] = C_max[i];
    G.print_break();
    return sol.size();
}


void pmcx_maxclique::branch_dense(
        vector<long long> &vs,
        vector<int> &es,
        vector <Vertex> &P,
        vector<short> &ind,
        vector<int> &C,
        vector<int> &C_max,
        vector <vector<int>> &colors,
        int *&pruned,
        int &mc,
        vector <vector<bool>> &adj) {

    // stop early if ub is reached
    if (not_reached_ub) {
        while (P.size() > 0) {
            // terminating condition
            if (C.size() + P.back().get_bound() > mc) {
                int v = P.back().get_id();
                C.push_back(v);
                vector <Vertex> R;
                R.reserve(P.size());

                for (int k = 0; k < P.size() - 1; k++)
                    // indicates neighbor AND pruned, since threads dynamically update it
                    if (adj[v][P[k].get_id()])
                        if ((*bound)[P[k].get_id()] > mc)
                            R.push_back(P[k]);


                if (R.size() > 0) {
                    // color graph induced by R and sort for O(1)
                    neigh_coloring_dense(vs, es, R, ind, C, C_max, colors, mc, adj);
                    branch_dense(vs, es, R, ind, C, C_max, colors, pruned, mc, adj);
                } else if (C.size() > mc) {
                    // obtain lock
#pragma omp critical (update_mc)
                    if (C.size() > mc) {
                        // ensure updated max is flushed
                        mc = C.size();
                        C_max = C;
                        print_mc_info(C, sec);
                        if (mc >= param_ub) {
                            not_reached_ub = false;
                            DEBUG_PRINTF("[pmc: upper bound reached]  omega = %i\n", mc);
                        }
                    }
                }
                // backtrack and search another branch
                R.clear();
                C.pop_back();
            } else return;
            P.pop_back();
        }
    }
}



