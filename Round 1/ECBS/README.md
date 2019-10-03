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

The resulting solution is guaranteed to be:
* conflict-free.
* $\sum l_i \leq w * opt$.
* $\max\{l_i} \leq makespan$.

where $l_i$ is the length of the path for agent $i$.

