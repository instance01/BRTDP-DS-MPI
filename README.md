# BRTDP-DS-MPI

DS-MPI is also called Dijkstra Sweep for Monotone Pessimistic Initialization [1].
It is used for finding an efficient upper bound initialization for BRTDP.
This repo includes an implementation of BRTDP and DS-MPI.

For all my homies out there struggling to implement DS-MPI.

Watch it visit only a small part of the whole state space!

![States visited by BRTDP](https://raw.githubusercontent.com/instance01/osmnx-mdp/master/.github/brtdp_band2.png)

## osmnx\_mdp

This is basically a stripped down version of [osmnx\_mdp](https://github.com/instance01/osmnx-mdp), which solves routing with uncertainty using Markov decision processes.

I decided to keep everything intact so it is clear how BRTDP is used. In this case, a single simulation of an agent trying to get from A to B on the Munich map using BRTDP.

Basically, `BRTDP\_DS\_MPI/algorithm/cpp\_brtdp.cpp` has the main implementation and `BRTDP\_DS\_MPI/simulation.pyx` applies BRTDP.

Running/Development:
```
sudo docker build -t brtdp-ds-mpi -f Dockerfile .
sudo docker run -v $(pwd):/app -it brtdp-ds-mpi bash
```

To run the current simulation:
```
cd BRTDP_DS_MPI
setup
run
```

[1] McMahan, H. Brendan, Maxim Likhachev, and Geoffrey J. Gordon. "Bounded real-time dynamic programming: RTDP with monotone upper bounds and performance guarantees." Proceedings of the 22nd international conference on Machine learning. ACM, 2005.
