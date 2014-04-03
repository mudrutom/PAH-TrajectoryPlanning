# PAH Trajectory Planning

Controlling of an agent [(UGV)](http://en.wikipedia.org/wiki/UGV) in the test simulation environment.
Its goal is to visit all provided checkpoints and to minimize the distance traveled.

It's the second assignment for the course Planning and Games (A4M36PAH) at FEE CTU.

## Implementation notes
* it uses the Rapidly-exploring Random Graph (RRG) algorithm to generate a node-graph used for finding a path
* a path between checkpoints is found by the A* search algorithm
* checkpoints are greedy-sorted to minimize a distance traveled
* new Visibility class with inflated buildings is used
* if the UGV gets stuck by a building it will turn sideways
* if the node-graph is disconnected, so a path couldn't be found, new near-checkpoint nodes are generated and added to the graph
