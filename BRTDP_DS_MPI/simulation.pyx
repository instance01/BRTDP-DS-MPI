# cython: language_level=3
import multiprocessing
import random
import pickle
from timeit import default_timer as timer

import numpy as np
import osmnx as ox

from BRTDP_DS_MPI.lib cimport draw_value_graph
from BRTDP_DS_MPI.lib cimport remove_dead_ends
from BRTDP_DS_MPI.lib cimport remove_zero_cost_loops
from BRTDP_DS_MPI.lib cimport get_time_to_drive
from BRTDP_DS_MPI.algorithm.mdp cimport MDP
from BRTDP_DS_MPI.algorithm.brtdp cimport BRTDP


# For each simulation this config is adapted.
# Algorithm configs always map 'bytestring -> double' due to static typing in
# C++.
CONFIG = {
    'processes': 1,
    'diverge_policy': 'random',  # Options: random, model
    'random_policy_percent': .2,
    'include_traffic_signals_in_metric': True,
    'MDP': {
        # Cython needs byte strings
        b'max_angle': 30,
        b'max_length': 200,
        b'edge_uncertainty': .1
    },
    'BRTDP': {
        b'alpha': 1e-20,
        b'tau': 100,
        b'heuristic': 1, # Options: 0 for Dijkstra, 1 for aerial distance
        b'heuristic_max_speed': 200 # In km/h
    }
}


MAPS = {
    'Munich': {
        'type': 'place',
        'file': 'data/munich.pickle',
        'place': 'Munich, Germany'
    }
}


LOCATIONS = {
    'Munich': [
        {
            'start': (48.164821, 11.3383748),
            'goal': (48.139404, 11.6708573),
            'metadata': {
                'id': 'AURIMUN',
                'info': 'Aubing to Riem through Munich'
            }
        },
        {
            'start': (48.2085997, 11.4586661),
            'goal': (48.1832655, 11.6284216),
            'metadata': {
                'id': 'M2',
                'info': ''
            }
        },
        {
            'start': (48.2085997, 11.4586661),
            'goal': (48.1180978, 11.5756126),
            'metadata': {
                'id': 'M3',
                'info': ''
            }
        },
        {
            'start': (48.1691419, 11.4148157),
            'goal': (48.1739931, 11.6512008),
            'metadata': {
                'id': 'M4',
                'info': ''
            }
        },
        {
            'start': (48.1451966, 11.5634101),
            'goal': (48.1785633, 11.471546),
            'metadata': {
                'id': 'M5',
                'info': ''
            }
        }
    ]
}


def load_map_from_osm(map_metadata):
    ox.config(use_cache=True)
    if map_metadata['type'] == 'place':
        G = ox.graph_from_place(
                map_metadata['place'],
                network_type='drive',
                which_result=map_metadata.get('which_result', 1),
                simplify=True)
    else:
        G = ox.graph_from_bbox(
                float(map_metadata['north']),
                float(map_metadata['south']),
                float(map_metadata['east']),
                float(map_metadata['west']),
                network_type='drive',
                simplify=True)
    G = ox.add_edge_lengths(G)
    return G


def _update_locs(G, map_id, type_='latlon'):
    # type_ can be latlon, then location currently has lat/lon coordinates,
    # or nxid, then location currently has a node id with x and y coordinates.
    for i, location in enumerate(LOCATIONS[map_id]):
        if type_ == 'latlon':
            start = location['start']
            goal = location['goal']
            method = 'haversine'
        else:
            start = (location['start']['y'], location['start']['x'])
            goal = (location['goal']['y'], location['goal']['x'])
            method = 'euclidean'

        LOCATIONS[map_id][i]['start'] = ox.utils.get_nearest_node(
                G,
                start,
                method=method)
        LOCATIONS[map_id][i]['goal'] = ox.utils.get_nearest_node(
                G,
                goal,
                method=method)
    return LOCATIONS[map_id]


def load_maps():
    for map_id, map_metadata in MAPS.items():
        try:
            with open(map_metadata['file'], 'rb') as f:
                locs, G = pickle.load(f)
        except Exception as ex:
            print('Failed loading map pickle for %s.' % map_id, ex)
            print('Loading fresh map from OSM.')
            G = load_map_from_osm(map_metadata)

            locs = _update_locs(G, map_id, 'latlon')

            print('Projecting graph. This might take a while..')
            G = ox.project_graph(G)

            for i, location in enumerate(LOCATIONS[map_id]):
                LOCATIONS[map_id][i]['start'] = G.nodes[location['start']]
                LOCATIONS[map_id][i]['goal'] = G.nodes[location['goal']]

            G2 = G.copy()
            print('Preprocessing graph (temporary)..')
            remove_zero_cost_loops(G2)
            remove_dead_ends(G2, None)

            locs = _update_locs(G2, map_id, 'nxid')

            with open(map_metadata['file'], 'wb+') as f:
                pickle.dump((locs, G), f)

        MAPS[map_id]['map'] = G
        LOCATIONS[map_id] = locs


def run_simulation(
        BRTDP algorithm,
        map_id,
        location_id,
        start,
        goal,
        diverge_policy,
        debug=False):
    """Run a simulation on a map using a specific algorithm.

    Algorithm shall be an algorithm class such as RTDP or D* Lite that
    implements the functions:
        setup()
        solve()
        drive(policy, diverge_policy)

    start and goal shall be in the form of (lat, lon)
    diverge_policy shall be a function in the form diverge_policy(G, node)
    and return for a given node and its options (successors) the next node.
    """
    algorithm_name = algorithm.__class__.__name__
    print("Running %s on map %s." % (algorithm_name, map_id))
    start_time = timer()

    algorithm.setup(start, goal, CONFIG[algorithm_name])
    policy = algorithm.solve()
    path = algorithm.drive(policy, diverge_policy)

    end_time = timer()

    total_time = end_time - start_time
    drive_time = get_time_to_drive(
            path,
            MAPS[map_id]['map'],
            CONFIG['include_traffic_signals_in_metric'])

    result = {
        'algorithm': algorithm_name,
        'id': location_id,
        'calculation_time': total_time,
        'drive_time': drive_time,
        'n_nodes': MAPS[map_id]['map'].number_of_nodes(),
        'path': path,
        'diverge_policy': diverge_policy,
        'iterations': algorithm.iterations
    }

    print("[%s, %s, %s] Seconds for calculation: %f" % (
        algorithm_name, map_id, location_id, total_time)
    )
    return result


def gen_diverge_policy(G, func):
    """Generate a diverge policy based on a general function.
    """
    policy = {}

    for node in G.nodes():
        if func(node):
            # Make sure successors are not already in diverge_policy, else we
            # might get a loop (two nodes diverging to each other in a loop).
            successors = [
                succ for succ in G.successors(node)
                if succ not in policy
            ]
            if len(successors) == 0:
                continue
            policy[node] = np.random.choice(successors, 1)[0]

    return policy


def gen_random_diverge_policy(G, density=.2):
    """Generate a divergence policy for all nodes:
    Diverge at roughly $(density) percent of nodes.
    We diverge by taking a random successor of the node.
    So this could also lie on the real path.
    """
    func = lambda _: random.random() < density
    return gen_diverge_policy(G, func)


def run_process(map_id, location):
    curr_result = []

    location_id = location['metadata']['id']

    start = location['start']
    goal = location['goal']

    G = MAPS[map_id]['map']

    print('Preprocessing graph..')
    remove_zero_cost_loops(G)
    remove_dead_ends(G, goal)

    print('filtered n_nodes/goal:', G.number_of_nodes(), goal)

    diverge_policy = gen_random_diverge_policy(G, CONFIG['random_policy_percent'])

    # Let's not diverge from start or goal.
    diverge_policy.pop(start, None)
    diverge_policy.pop(goal, None)

    mdp = MDP(G)
    mdp.setup(start, goal, CONFIG['MDP'])
    brtdp = BRTDP(mdp)
    curr_result.append(run_simulation(brtdp, map_id, location_id, start, goal, diverge_policy))

    return curr_result


def run_simulations(iterations=1):
    start_time = timer()

    load_maps()
    
    results = []

    for map_id in MAPS.keys():
        for location in LOCATIONS[map_id]:
            results.extend(run_process(map_id, location))

    # Save results to simulation.pickle after each iteration so if it should
    # ever crash, we have partial results.
    # The point is, they're only improving slightly per iteration, so these
    # 'partial' results would still be very usable.
    print('Saving temporary results to simulation.pickle..')
    with open('simulation.pickle', 'wb+') as f:
        pickle.dump(results, f)

    print('Total time for simulation:', timer() - start_time)


run_simulations()
