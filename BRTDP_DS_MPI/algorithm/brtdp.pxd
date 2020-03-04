# cython: language_level=3
from libcpp.unordered_map cimport unordered_map
from libcpp.string cimport string
from libcpp.vector cimport vector
from libcpp.pair cimport pair
from BRTDP_DS_MPI.algorithm.dense_hash_map cimport dense_hash_map
from BRTDP_DS_MPI.algorithm.mdp cimport MDP


cdef extern from "cpp_brtdp.hpp":
    struct pair_hash:
        long operator(pair[long, long])

    cppclass CPP_BRTDP "BRTDP":
        CPP_BRTDP()

        dense_hash_map[pair[long, long], double, pair_hash] vl

        int init(
            vector[long] *S,
            dense_hash_map[long, vector[pair[long, long]]] *A,
            dense_hash_map[pair[long, long], double, pair_hash] *C,
            dense_hash_map[
                long,
                dense_hash_map[
                    pair[long, long],
                    vector[pair[long, double]],
                    pair_hash
                ],
            ] *P,
            dense_hash_map[long, vector[long]] *predecessors,
            dense_hash_map[long, pair[double, double]] *data
        )
        int setup(long start, long goal, unordered_map[string, double] cfg)
        int run_trials()
        vector[long] get_path(dense_hash_map[long, long] diverge_policy)


cdef class BRTDP:
    cdef dense_hash_map[long, vector[long]] predecessors
    cdef dense_hash_map[long, pair[double, double]] data
    cdef dense_hash_map[pair[long, long], double, pair_hash] vl

    cdef MDP mdp
    cdef CPP_BRTDP cpp
    cdef int iterations

    cdef setup(self, long start, long goal, unordered_map[string, double] cfg)
    cdef run_trials(self)
    cdef get_path(self, diverge_policy)
    cdef solve(self)
    cdef drive(self, policy, diverge_policy)
