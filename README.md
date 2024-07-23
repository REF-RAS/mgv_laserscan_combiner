# Laserscan and Pointcloud Combiner

![QUT REF Collection](https://badgen.net/badge/collections/QUT%20REF-RAS?icon=github) [![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)

**Robotics and Autonomous Systems Group, Research Engineering Facility, Research Infrastructure**
**Queensland University of Technology**

## Introduction

The  Laserscan and Pointcloud Combiner is a ROS 1 package that is used to combine PointCloud2 and LaserScan input sources by selecting the nearest detection. The resulting detection is published to a LaserScan topic.  The package supports at most 1 PointCloud2 source and 2 LaserScan sources. Due to resampling, quantization error may be added in the processing.

![Figure](docs/assets/DataFlow.png)

More details will be provided.

![Figure](docs/assets/LaserScanDemo.png)


## Developer

Dr Andrew Lui, Senior Research Engineer <br />
Robotics and Autonomous Systems, Research Engineering Facility <br />
Research Infrastructure <br />
Queensland University of Technology <br />

First version: 23 Jul 2024 <br />
Latest update: Jul 2024