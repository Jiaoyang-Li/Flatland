#!/bin/bash

cmake ./CBS-corridor -DCMAKE_BUILD_TYPE=Release &&\
make &&\
cp ./PythonCBS/libPythonCBS.so ./libPythonCBS.so &&\
python ./run.py
