# Dependency

**Boost 1.61.0**
**Boost.python36**
**Python 3.6**

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
plan is a list of list, which stores paths of all agents.
