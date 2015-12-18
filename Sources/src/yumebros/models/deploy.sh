#!/bin/bash

models="$HOME/.gazebo/models"

for folder in $(find . -mindepth 1 -maxdepth 1 -type d)
do
  rm -vfR "$models/$folder"
  cp -vR "$folder" "$models"
done
