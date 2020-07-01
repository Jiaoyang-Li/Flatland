# Dependency

1. Downgrade your python to 3.6 and make sure python-dev is also installed.

2. Install boost 1.61.0 (must include libboost-python3)
    * Follow Section 5 in [boost 1.61.0 document](https://www.boost.org/doc/libs/1_61_0/more/getting_started/unix-variants.html) to install the library;
    * In particular, when you run bootstrap.sh, make sure it finds the correct version of python. If you didn't see "Detecting Python version... 3.6", use bootstrap.sh --help to learn to configure path to your python manually.
    * After installation, make sure that libboost-python3 is in you boost library (which is located at /usr/local/lib by defalult). You might find the library with a slightly different name (e.g., libboost-python36 or libboost-python-py36), in which case, you need to replace "python3" with the last part of the name of your library for variable boostPython in both PythonCBS\CMakelists.txt and CBSH-rect-cmake\CMakeLists.txt. For example, change "set(boostPython python3)" to "set(boostPython python36)".
3. If you are using windows, configure paths of dependencies manually.

# Usage

Compile codes and make sure libPythonCBS.xx is compiled at the folder where your python codes are.

Then, in python codes:

```python
from libPythonCBS import PythonCBS
f_w = 1
debug = True
k = 1
timelimit = 240  # unit: seconds
default_group_size = 16 # max number of agents in a group
corridor_method = "trainCorridor1" # or "corridor2" or ""
CBS = PythonCBS(env,"ICBS",k,timelimit,default_group_size,debug,f_w,corridor_method)
success = CBS.search()
plan = CBS.getResult()
```

corridor_method can be "trainCorridor1" or "corridor2" or "". 
"trainCorridor1" is more greedy which don't consider bypass paths of a corridor.
"corridor2" considerd bypass paths.
"" turn off corridor reasoning.

Success is a boolean, which indicate does the search success.

plan is a list of list, which stores paths of all agents. 

The format of a path is \[-1, -1,200,234,345\]. -1 indicate the train is not active.

run_test2.2.py contains a test example.