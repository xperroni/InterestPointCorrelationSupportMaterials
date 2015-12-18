#!/bin/bash

MODE=Release

if [ "$#" -eq 1 ]
then
  MODE=$1
fi

echo $MODE

catkin_make_isolated -DCMAKE_BUILD_TYPE=$MODE
