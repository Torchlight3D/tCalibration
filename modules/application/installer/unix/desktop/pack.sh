#!/bin/sh
exe="CalibrationTool"
des="$HOME/Projects_build/CalibrationTool"
deplist=$(ldd $exe | awk '{if (match($3, "/")){ printf("%s "),$3}}')
cp $deplist $des