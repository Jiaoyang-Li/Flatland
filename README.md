# Multi-Agent Path Finding for Large-Scale Rail Planning

<p align="center">
  <img src="./solution.gif" />
</p>

## Introduction

[NeurIPS 2020 Flatland Challenge](https://www.aicrowd.com/challenges/neurips-2020-flatland-challenge) 
is a railway scheduling competition which was held in partnership with German, Swiss, and French railway companies. 
This repository contains the **winner solution** from the team An_Old_Driver.

The organizers characterized the research challenge, that lasted several months with continuous software submissions,
as follows: 
>This challenge tackles a key problem in the transportation world: How to efficiently manage dense traffic 
on complex railway networks? This is a real-world problem faced by many transportation and logistics companies around 
the world such as the Swiss Federal Railways and Deutsche Bahn. Your contribution may shape the way modern traffic management
systems are implemented, not only in railway but also in other areas of transportation and logistics.

The software is based on multi-agent path finding (MAPF) technology and has reached the highest score in both rounds of the challenge and outperformed all other entries in both tracks, including all reinforcement learning entries. According to the organizers, there were more than 700 participants from 51 countries making more than 2,000 submissions. 

Please check our paper [1] for more details.

[1] Jiaoyang Li, Zhe Chen, Yi Zheng, Shao-Hung Chan, Daniel Harabor, Peter J. Stuckey, Hang Ma and Sven Koenig. Scalable Rail Planning and Replanning: Winning the 2020 Flatland Challenge. In Proceedings of the International Conference on Automated Planning and Scheduling (ICAPS), pages 477-485, 2021.

## Credits
The software is developed by
* Jiaoyang Li, University of Southern California
* Zhe Chen, Monash University
* Yi Zheng, University of Southern California
* Shao-Hung Chan, University of Southern California

We would like to thank to Han Zhang for initially tried some ideas for the competition. 
We would like to thank Daniel Harabor, Peter J. Stuckey, Hang Ma and Sven Koenig for their ideas and advice.

Copyright (c) 2020 The University of Southern California. All Rights Reserved.

Copyrights licensed under an Academic/non-profit use license.

See the accompanying LICENSE file for terms.

**In addition to the above license,  this software is not allowed to be used in 2021 or later flatland 
challenges or any other challenges.**

## Dependency

The software is developed in C++ and uses boost-python to interact with the Python-based Flatland simulator. 
boost-python requires special versions of boost and python, and therefore, 
we suggest you follow the following instruction to install boost and boost-python:

1. Downgrade your python to 3.6 and make sure python-dev is also installed.

2. Install boost 1.61.0 (must include libboost-python3) 
   (Other versions of boost may also work. However you need to make sure your boost-python matches the python version 
    and configure CMakeLists.txt to let the compiler find right packages.)
    * Follow Section 5 in [boost 1.61.0 document](https://www.boost.org/doc/libs/1_61_0/more/getting_started/unix-variants.html) to install the library;
    * In particular, when you run bootstrap.sh, make sure it finds the correct version of python. If you didn't see "Detecting Python version... 3.6", use bootstrap.sh --help to learn to configure path to your python manually.
    * After installation, make sure that libboost-python3 is in you boost library (which is located at /usr/local/lib by defalult). You might find the library with a slightly different name (e.g., libboost-python36 or libboost-python-py36), in which case, you need to replace "python3" with the last part of the name of your library for variable boostPython in both PythonCBS\CMakelists.txt and CBSH-rect-cmake\CMakeLists.txt. For example, change "set(boostPython python3)" to "set(boostPython python36)".
3. If you are using Windows, configure paths of dependencies manually in ./Mapf-solver/CMakeLists.txt.

## Usage

### Testing on your local machine
Compile codes under Mapf-solver using cmake and make sure libPythonCBS.xx is compiled at the folder where your python codes are.
* ./run_example.py is an example pythoin code that we use to test our software on a locally generated flatland problem.
* Below is a template python code with explanations:

```python
# import mapf solver from shared lib
from libPythonCBS import PythonCBS

# Parameter initialization
agent_priority_strategy = 3  #  the strategy for sorting agents, choosing a number between 0 and 5
#                               0: keep the original ordering
#                               1: prefer max speed then max distance
#                               2: prefer min speed then max distance
#                               3: prefer max speed then min distance
#                               4: prefer min speed then min distance
#                               5: prefer different start locations then max speed then max distance
neighbor_generation_strategy = 3    # 0: random walk; 1: start; 2: intersection; 3: adaptive; 4: iterative
debug = False
framework = "LNS"  # "LNS" for large neighborhood search or "Parallel-LNS" for parallel LNS.
time_limit = 200  #Time limit for computing initial solution.
default_group_size = 5  # max number of agents in a group for LNS
stop_threshold = 30
max_iteration = 1000 # when set to 0, the algorithm only run prioritized planning for initial solution.
agent_percentage = 1.1 # >1 to plan all agents. Otherwise plan only certain percentage of agents.
replan = True # turn on/off partial replanning.
replan_timelimit = 3.0 # Time limit for replanning.

# Initialize local flatland environment
local_env = ......

# Search for solution
solver = PythonCBS(local_env, framework, time_limit, default_group_size, debug, replan,stop_threshold,agent_priority_strategy,neighbor_generation_strategy)
solver.search(agent_percentage, max_iteration)

# Build MCP
solver.buildMCP()

# Then in the main while loop of the Flatland simulator
# Get corresponding action dictionary by:
action = solver.getActions(local_env, steps, replan_timelimit) # steps: current timestep
```


* Other important hard-coded parameters are in ./Mapf-solver/PythonAPI/PythonCBS.h:
```c++
int max_replan_times = 50000; // Maximum replanning times.
float max_replan_runtime = 100; // Maximum time spend on replanning.
int strategies[4] = {1,3,5,6}; // agent_priority_strategies for parallel-lns.
```
### Submitting to the Flatland contest server.
* Example run.py script is located in folder ./Flatland2020SubmissionKit
* Place Mapf-solver folder under the root of submission repo. Make sure run.sh can find Mapf-solver for compiling source code.
* Dependencies required by docker are described in ./Flatland2020SubmissionKit/apt.txt 
* Following the official submission guide to make submissions.

<!---# Algorithm Overview

## MAPF Model
Here is a summary of the changes to the standard MAPF model:
* Agents do not appear on the map before they start to move and disappear from the map after reaching their goal locations.
* All agents have the same speed.
* The orientations of the agents are considered. In most cases (unless hitting a deadend), the agents cannot move backwards.
* Agents need to reach their goal locations before a given max timestep. 
* Agents may meet malfunction during plan execuatation.

## Framework 1: LNS
We use Large Neighbourhood Search (LNS) as the framwork. 

```c++
A = [a1, a2, ...];  // unplanned agents
A = sort(A);  // sort the agents by some heuristics (e.g., speed, distance to the goal location)
P = PrioritizedPlanning(A); // get initial paths by PP
while(not timeout and m <= num_of_agents) {
    A' = a subset of agents in A;
    old_paths = paths of A' in P;
    remove old_paths from P;
    paths = replan paths for agents in A' by viewing paths in P as dynamic obstabcles;
    if (paths are better than old_paths)
        add paths to P;
    else
        add old_paths to P;
}
```

### Different stratefies for the neighborhood search

#### Strategy 1: random walk
We select a bottleneck agent and let it perform a random walk on "improving moves" until it conflicts with m-1 agents.
Together with the bottle neck agent, we replan the paths of the m agents by CBS.
```c++
a = the agent with max cost increment in A but not in tabu_list;
update tabu_list;
conflicting_agents = randomWalk(a, m - 1); // let agent a perform a random walk (starting from a random timestep on its path) until it conflicts with m - 1 agents
A' = {a} + conflicting_agents;
old_paths = paths of A' in P;
remove old_paths from P;
T = min(remaining_runtime, 10s);
paths = CBS(A', T, P);  // try to find collision-free paths for agents in a that do not collide with any path in P by CBS with a time limit of T 
if(paths are found) {
    add paths to P;
    m += 1;
} else {
    add old_paths to P;
    m -= 1;
}
```

#### Strategy 2: start location
We randomly select a start location and collect the agents who start from there.
We sort the agents in decreasing order of their cost increments (breaking ties randomly) and replan their paths by prioritized planning.
 ```c++
x = a random start location;
A' = agents who start at location x;
A' = sort(A'); // sort the agents in decreasing order of their cost increments
old_paths = paths of A' in P;
remove old_paths from P;
paths = PrioritizedPlanning(A); // get initial paths by PP
if (paths are better than old_paths)
    add paths to P;
else
    add old_paths to P;
 ```

#### Strategy 3: intersection
We randomly select an intersection and collect the agents who has visited there.
We sort the agents randomly and replan their paths by prioritized planning.
 ```c++
x = a random inersection location;
A' = agents who has visited x;
A' = random_shuffle(A');
old_paths = paths of A' in P;
remove old_paths from P;
paths = PrioritizedPlanning(A); // get initial paths by PP
if (paths are better than old_paths)
    add paths to P;
else
    add old_paths to P;
 ```


## Framework 2: CPR

The idea is borrowed from Complete Path Reservation (CPR) from one of the last year's 
[solution](https://eprints.hsr.ch/855/1/Masterarbeit_Waelter_Jonas.pdf).
The idea is that we dynamically assign directions to the edges on the graph so that, 
at any timestep, for every pair of neighboring locations v and u, we only allow agents to move in one direction, 
i.e., either from v to u or from u to v.
If we ask all agents to follow their shortest paths on this modified directed graph, it is guaranteed that 
all agents that have paths to their goal locations on the directed graph 
can reach their goal locations without deadlocks.

We use a matrix ``highways'' of size (map-size * map-size) to represent the directions of the edges. 
At any timestep, for any pair of locations u and v, highways[u][v] + highways[v][u] = 0.
- highways[u][v] = k > 0 means that there are currently k agents that *want to* move from u to v. 
  Future agents can also move from u to v.
- highways[u][v] = -k < 0 means that there are currently k agents that *want to* move from v to u. 
  So future agents cannot move from u to v.
- highways[u][v] = 0 means that there are no agents that *want to* move from u to v or v to u. 
  So the next agent can move either from u to v or v to u.
  
We update highways whenever we plan a path for an agent and whenever an agent moves.
- If a new planned path traverses edge (u, v), then highways[u][v]++ and highways[v][u]--.
- If an agent moves from u to v at the current timestep, then highways[u][v]-- and highways[v][u]++,
  i.e., the agent has already used this edge, so it can release its reservation in highways.
  
When we plan path for an agent, we can only use edges (u, v) that satisfy highways[u][v] >= 0. 

We show the pseudo-codes below.

### Initial Planning
```c++
A = [a1, a2, ..., am];  // unplanned agents
P = {};  // planned paths
A = sort(A);  // sort the agents by some heuristics
highways = a (map-size * map-size) zero matrix; // the directions of the edges
for (a in A) {
    path = A*(a, highways);  // find a shortest path by A* that only uses edges (u, v) that satisfy highway[u][v] >= 0
    if(path is found) {
        for ((u,v) in path) { // update highways
            highways[u][v]++;
            highways[v][u]--;
        }
        Add path to P;
        Remove a from A;
    }
}
```

### Replanning at every timestep during execution
agent_steps is intialized as a zero vector of length m (m is the number of agents) at timestep 0. 
It represents the number of steps each agent has already taken along its path.

```c++
/* Update agent_steps and highways */
currs = locations of all agents at the current timestep;
prevs = locations of all agents at the previous timestep;
replan = false;
for (i = 1; i < m; i ++) {  // m is the number of agents
    if (currs[i] != prevs[i]) {  // agent i moves from prevs[i] to currs[i]       
        agent_steps[i]++;
        highways[prevs[i]][currs[i]]--;
        highways[currs[i]][prevs[i]]++;       
        if (highways[prevs[i]][currs[i]] == 0) {
            replan = true;
        }
    }
}

/* replan if necessary*/
if (replan) {
    for (a in A) {
        path = A*(a, highways);  // find a shortest path by A* that only uses edges (u, v) that satisfy highway[u][v] >= 0
        if(path is found) {
            for ((u,v) in path) { // update highways
                highways[u][v]++;
                highways[v][u]--;
            }
            Add path to P;
            Remove a from A;
        }        
    }
}

/* generate the next location that each agent needs to go */
togo = vector(m, nullptr);
for (i = 1; i < m; i ++) {
    p = the path of agent ai in P;
    if (p exists and agent ai has not reach its goal location) {
        togo[i] = p[agent_steps[i] + 1];
    }
}
return togo;
```




# Past Winner Solutions

Here are some material about the winner solutions of last year.


| Team | Initial planning | Replanning | Success rate in the final round |
| --- | --- | --- | --- |
|First place| Prioritized planning (max-speed agent first, breaking ties randomly) with multiple runs | MCP + prioritized planning   | 99% |
|[Second place](https://docs.google.com/presentation/d/12bbp7MwoB0S7FaTYI4QOAKMoijf_f4em64VkoUtdwts/edit#slide=id.g6dde6a5360_0_1)| Prioritized planning  (max-speed agent first, breaking ties by preferring min-distance agent) | MCP | 96% |
|[Third place](https://github.com/vetand/FlatlandChallenge2019/blob/master/Approach_description.pdf)| Prioritized planning  (max-speed agent first) | Replan the delayed agent by viewing it as the lowest-priority agent | 95% |
|[Fourth place](https://eprints.hsr.ch/855/1/Masterarbeit_Waelter_Jonas.pdf)| Prioritized planning (max-distance agent first) | Complete Path Reservation (CPR) or reinforcement learning| 79% |
|Fifth place| Reinforcement learning | Reinforcement learning | 55% |

There are also some [presentations](https://www.youtube.com/watch?v=rGzXsOC7qXg) available online.>
-->

# Further Reading

## ICAPS Best Demonstration Award Video
[![Demo Video](https://img.youtube.com/vi/Pw4GBL1UhPA/0.jpg)](https://www.youtube.com/watch?v=Pw4GBL1UhPA)

## Presentation on our solution
[![Solution Talk](video_cover.png)](https://www.youtube.com/watch?v=pNbFDVXkHQ0)




More details of the vairous MAPF technologies mentioned in the talk and other MAPF related material can be found at [mapf.info](http://mapf.info/).

## 2019 Flatland Challenge
Solutions from [2019 Flatland Challenge](https://www.aicrowd.com/challenges/flatland-challenge)
* [First place](https://www.aicrowd.com/blogs/flatland-mugurel)
* [Second place](https://docs.google.com/presentation/d/12bbp7MwoB0S7FaTYI4QOAKMoijf_f4em64VkoUtdwts/edit#slide=id.g6dde6a5360_0_1)
* [Third place](https://github.com/vetand/FlatlandChallenge2019/blob/master/Approach_description.pdf)
* [Fourth place](https://eprints.hsr.ch/855/1/Masterarbeit_Waelter_Jonas.pdf)

[Flatland 2019 Presentation](https://www.youtube.com/watch?v=rGzXsOC7qXg)
