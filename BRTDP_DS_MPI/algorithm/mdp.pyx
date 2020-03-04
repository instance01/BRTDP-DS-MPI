# cython: language_level=3
# cython: profile=True
from libcpp.pair cimport pair
from libcpp.vector cimport vector

from BRTDP_DS_MPI.lib cimport get_edge_cost


cdef class MDP:
    def __init__(self, G):
        self.G = G

    cdef _setup_cpp(self):
        self.predecessors.set_empty_key(0)
        self.successors.set_empty_key(0)
        self.node_data.set_empty_key(0)
        self.edge_data.set_empty_key((0, 0))
        self.angle_data.set_empty_key((0, 0))
        self.A.set_empty_key(0)
        self.P.set_empty_key(0)
        self.C.set_empty_key((0, 0))

        for node in self.G.nodes():
            self.P[node].set_empty_key((0, 0))
        self.P[1].set_empty_key((0, 0))

        for node in self.G.nodes.data():
            node_id = node[0]
            successors = list(self.G.successors(node_id))
            predecessors = list(self.G.predecessors(node_id))

            self.predecessors[node_id] = predecessors
            self.successors[node_id] = successors

            self.node_data[node_id] = (node[1]['x'], node[1]['y'])

            self.S.push_back(node_id)
            self.A[node_id] = [(node_id, succ) for succ in successors]

            for succ in successors:
                action = (node_id, succ)
                # For now we end up in correct state 100% of the time.
                if len(predecessors) == 0:
                    self.P[1][action].push_back((succ, 1.0))
                    self.predecessors[node_id].push_back(1)
                for pred in predecessors:
                    self.P[pred][action].push_back((succ, 1.0))
                self.C[action] = get_edge_cost(self.G, node_id, succ)

                has_traffic_signal = self.G.nodes[succ]['highway'] == 'traffic_signals'
                if has_traffic_signal:
                    # 5 seconds converted to hours.
                    self.C[action] += 5 / 3600.

                self.edge_data[(node_id, succ)] = self.G[node_id][succ][0]['length']

                data = self.G[node_id][succ][0]
                if not data.get('geometry'):
                    real_coords = (self.G.nodes[succ]['x'], self.G.nodes[succ]['y'])
                else:
                    real_coords = list(data['geometry'].coords)[1]
                self.angle_data[(node_id, succ)] = real_coords

    cdef setup(self, long start, long goal, unordered_map[string, double] cfg):
        self._setup_cpp()
