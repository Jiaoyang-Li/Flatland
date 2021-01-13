#!/bin/bash

cmake ./Mapf-solver -DCMAKE_BUILD_TYPE=Release &&\
make &&\
cp ./libPythonCBS.so ./libPythonCBS.so &&\
python ./run.py
