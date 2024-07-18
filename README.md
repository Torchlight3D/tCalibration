CalibKit
===

_This is a Torchlight3D project_

## Motivation

Sensor calibration is a critical task in numerous fields such as robotics, computer vision, and augmented reality. Despite its apparent simplicity, achieving high-quality calibration is complex and often requires solutions tailored to specific hardware configurations. This complexity arises from the need to precisely align sensor data in multi-sensor systems, ensuring accurate data fusion and reliable performance.

The process of sensor calibration shares many similarities with 3D reconstruction and Simultaneous Localization and Mapping (SLAM). These tasks often utilize common mathematical models, optimization techniques, and algorithms. By leveraging these shared modules, we can create a more efficient and cohesive calibration library.

Existing libraries like Kalibr and OpenCV have made significant strides in sensor calibration. However, they fall short in terms of customizability and portability. These tools often provide robust solutions for standard configurations but lack the flexibility required for unique or evolving hardware setups. This limitation can be a significant barrier for researchers and developers working with non-standard or cutting-edge sensor arrays.

Moreover, there has been a surge of state-of-the-art research in sensor calibration, often open-sourced or otherwise accessible. While these contributions are invaluable, they are frequently developed by students and researchers whose primary focus is on academic exploration rather than production-level implementation. As a result, these projects may not be well-organized or thoroughly tested to meet the rigorous standards of real-world applications.

Our motivation for developing this new calibration library is to address these gaps. We aim to create a tool that not only achieves high accuracy but also offers extensive customization options and portability. By integrating modular components that can be easily adapted and extended, we hope to support a wide range of hardware configurations and use cases. Furthermore, we intend to consolidate and refine cutting-edge research contributions, ensuring they are organized, tested, and ready for practical deployment.

This project is driven by the need for a more versatile and adaptable calibration solution that meets the demands of modern multi-sensor systems. By open-sourcing our library, we invite the community to contribute, enhance, and tailor it to their specific needs, fostering innovation and collaboration in sensor calibration and related fields.

## Goals

+ Modern

+ Modular

+ Well-tested

+ Portable

+ Cross-platform

## Build

### Dependencies

A complete 3rd-party library list can be found [here](DEPS.md).

Most of the dependencies are the same on both Windows and Linux.

+ OpenCV

+ Eigen

+ Google Suite (gflags, glog, googletest)

+ ceres-solver

+ fmt

+ Sophus

+ Qt (optional)

+ ROS (optional)

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

## RoadMap

### Algorithms

+ [ ] Merge camera calibration by reconstruction
+ [ ] Add photometric camera calibration
+ [ ] Add more geometric camera models
+ [ ] Use CRTP on Camera class
+ [ ] Add Stag and CCTag, complete MultiChessboardDetector
+ [ ] Add Handeye calibration
+ [ ] Better IMU calibration, make use of B-spline
+ [ ] Complete MTF
+ [ ] Complete structured light
+ [ ] Bring in deep learning, start with feature detection

### Software

+ [ ] Move application and Qt related stuffs out of `modules`
+ [ ] Better CMake, library export and package
+ [ ] Documentations
+ [ ] Add Python binding (pybind11)
+ [ ] Add 3D visualization module (Complete the old Qt-based one, or start over with imgui?)
+ [ ] Open sourced project desciption, setting, and license stuffs 