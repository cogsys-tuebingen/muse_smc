#!/bin/bash
catkin_make clean && catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_CXX_FLAGS=-Ofast 
