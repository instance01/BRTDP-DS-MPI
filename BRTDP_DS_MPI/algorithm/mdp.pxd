# cython: language_level=3
from libcpp.string cimport string
from libcpp.pair cimport pair
from libcpp.vector cimport vector
from libcpp.unordered_map cimport unordered_map
from libcpp.set cimport set as cset

from BRTDP_DS_MPI.algorithm.dense_hash_map cimport dense_hash_map


cdef extern from "../cpp_lib.hpp":
    struct pair_hash:
        long operator(pair[long, long])

cdef class MDP:
    cdef vector[long] S
    cdef dense_hash_map[long, vector[pair[long, long]]] A
    cdef dense_hash_map[pair[long, long], double, pair_hash] C
    cdef dense_hash_map[
        long,
        dense_hash_map[
            pair[long, long],
            vector[pair[long, double]],
            pair_hash
        ]
    ] P

    cdef dense_hash_map[pair[long, long], double, pair_hash] edge_data
    cdef dense_hash_map[long, pair[double, double]] node_data
    cdef dense_hash_map[pair[long, long], pair[double, double], pair_hash] angle_data
    cdef dense_hash_map[long, vector[long]] predecessors
    cdef dense_hash_map[long, vector[long]] successors

    #cdef CPP_MDP cpp
    cdef G

    cdef long start
    cdef long goal

    cdef close_nodes
    cdef angle_nodes

    cdef set uncertain_nodes

    cdef _setup_cpp(self)
    cdef setup(self, long start, long goal, unordered_map[string, double] cfg)
