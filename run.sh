#!/bin/bash

cmake ./CBS-corridor -DCMAKE_BUILD_TYPE=Release &&\
make -j 4 &&\
cp ./PythonCBS/libPythonCBS.so ./libPythonCBS.so &&\
echo "cmake success" &&\
python ./run.py
