# ECBS

The code currently is tested on the MAPF benchmark instances.

The code requires the external library: BOOST (https://www.boost.org/).

To compile the code:
```
cmake .
make
```

To run the code:
```
./ECBS -m instances/lak503d.map  -a instances/lak503dmap-100agents-1.agents -o test.csv -t 60 -w 1.2
```

## To do list:
* Use a better data structure for constraint table.
* Replace Grid2d with a new graph class designed for Flatland, and change the code accordingly (i.e., the duplicate detection for the single-agent search).
* Change the high-level focal search so that it prioritizes the CT node with the minimum number of collisions and break ties by the sum of costs.

