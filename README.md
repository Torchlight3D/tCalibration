CalibKit
===

## Introduction

## Build

### Dependencies

Most of the dependencies are the same on both Windows and Linux.

+ OpenCV

+ Eigen

+ Google Suite (gflags, glog, googletest)

+ ceres-solver

+ fmt

+ Sophus

+ Qt (optional)

+ ROS (optional)

+ Mynt Depth SDK (optional)

My sugguestion on managing source-built external libraries is to put 
all the built results in a specific directory. A preferable external 
root could be

+ Windows: D:/Dependencies
+ Linux: ${HOME}/Dependencies (e.g. /home/bobblelaw/Dependencies)

Take OpenCV for example, say you try to build the latest OpenCV source.
Then the built targets should be found in

+ Windows: D:/Dependencies/opencv/4.7.0n
+ Linux: ${HOME}/Dependencies/opencv/4.7.0n

For the suffix, you can have your own style. Like "-stable", "-dev", 
or "n" stands for nightly, "x" stands for extra, etc. In real production 
scenario, we **MUST** choose stable version.

## Wiki