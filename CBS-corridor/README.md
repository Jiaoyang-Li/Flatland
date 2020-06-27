# Dependency

1. Downgrade your python to 3.6 and make sure python-dev is also installed.

2. Install boost 1.61.0 (must include libboost-python3)

3. If you are using windows, configure paths of dependencies manually.

# Usage

Compile codes and make sure libPythonCBS.xx is compiled at the folder where your python codes are.

Then, in python codes:

```
from libPythonCBS import PythonCBS
f_w = 1
debug = True
k = 1
timelimit = 10
CBS = PythonCBS(env,"ICBS",k,timelimit,debug,f_w,"trainCorridor1")
success = CBS.search()
plan = CBS.getResult()
```
success is a boolean, which indicate does the search success.
plan is a list of list, which stores paths of all agents. The format of a path is \[-1, -1,200,234,345\]. -1 indicate the train is not active.
