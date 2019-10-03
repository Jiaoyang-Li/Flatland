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
./ECBS -m ../map.txt  -a ../agents.txt -o ../paths.txt -t 60 -w 1.2
```

## To do list:
* Use a better data structure for constraint table.
* Change the high-level focal search so that it prioritizes the CT node with the minimum number of collisions and break ties by the sum of costs.

