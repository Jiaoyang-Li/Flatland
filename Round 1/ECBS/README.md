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
./ECBS -m instances/lak503d.map  -a instances/ak503dmap-100agents-1.agents -o test.csv -t 60 -w 1.2
```

## To do list:
* Replace Grid2d with a new graph class designed for Flatland, and change the code accordingly.


