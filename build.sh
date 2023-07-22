#!/bin/bash
cd src/utils/include/lmbm && make -j 20 && make clean &&sleep 1
cd ../../../../
catkin_make -j20 -DPYTHON_EXECUTABLE=/usr/bin/python3