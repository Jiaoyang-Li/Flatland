# ECBS

The code requires the external library: BOOST (https://www.boost.org/).

To compile the code:
```
cmake .
make
```

To run the code:
```
./ECBS -m ../map.txt  -a ../agents.txt -o ../paths.txt -t 60 -w 1.2 --makespan 1000
```
NOTE: 
The above cmd is for running from ./ECBS folder. If run the code from a different folder, please relocate the agents.txt, map.txt files.


The resulting solution is guaranteed to be:
* conflict-free.
* $\sum l_i \leq w * opt$.
* $\max l_i \leq makespan$.

where $l_i$ is the length of the path for agent $i$.

Required inputs:

  -m [ --map ]    : input file for map
  
  -a [ --agents ] : input file for agents
  
  -o [ --output ] : output file for schedule
  
  -w [ --weight ] : suboptimal bound for ECBS
 
 Optional inputs:
  
  --makespan          : maximum makespan, with default value of 2147483647
  
  -d [ --disjoint ]   : disjoint splitting, with the default value of False
  
  -t [ --cutoffTime ] : cutoff time (seconds), with the default value of 300
  
  --seed              : random seed, with the default value of 0 
  
  --debug             : debug mode, with the default value of False
  
  --help              : produce help message
