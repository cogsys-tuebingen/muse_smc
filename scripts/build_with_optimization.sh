#!/bin/bash
if [ "$1" = "--clean" ] 
	then
		catkin_make clean
fi
catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_CXX_FLAGS=-Ofast 
