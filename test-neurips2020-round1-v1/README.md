# Test instances and paths for Round 1

``load.py`` contains a template code to load the instances and the paths.

The format of a path is \[-1, -1,200,234,345\]. -1 indicate the train is not active.
If a path is an empty list \[\], 
it is either we cannot find a path for the train to reach its goal location before the deadline 
or we do not have time to plan its path. But for the current instances, we successfullt found paths for all agents. 
