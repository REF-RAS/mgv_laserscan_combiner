# Laserscan and Pointcloud Combiner

![QUT REF Collection](https://badgen.net/badge/collections/QUT%20REF-RAS?icon=github) [![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)

**Robotics and Autonomous Systems Group, Research Engineering Facility, Research Infrastructure**
**Queensland University of Technology**

## Introduction

The Laserscan and Pointcloud Combiner is a ROS 1 node that is designed for combining PointCloud2 and LaserScan input sources by selecting the nearest detection in a ROS application. The resulting detection is published to a LaserScan topic.  The package supports at most 1 PointCloud2 source and 2 LaserScan sources. Due to resampling, quantization error may be introduced during the processing of combining multiple sources of different sampling patterns.

## ROS Integration

The following figure illustrates the data flow between the Combiner and the ROS 1 infrastructure.

![Figure](docs/assets/DataFlow.png)

The input to the Combiner may include 1 to 3 sources, with at most one of them a `PointCloud2` message topic. The others are `LaserScan` message topics. The `LaserScan` topics may be of different sampling patterns (i.e., sampling rate and sample angle range). The `PointCloud2` input will pass through a filter that selects data in a specified z-axis range. This enables the selection of objects at different vertical positions and the ignoring of the ground plane.

The Combiner operates at a specified sampling pattern, which are specified as parameters. If the input sources include one `LaserScan` message topic, then the topic's sampling pattern may be adopted as the default output sampling pattern parameter.

The Combiner combines the input sources through selecting the nearest data point for every output sample. In the case of a `PointCloud2` input message topic, the Combiner will convert the pointcloud data into laser scan data by quantization. This involves selecting the nearest point within a quantized angle range and z-axis range. In the case of a `LaserScan` input message topic, the Combiner will resample and again select the nearest point when there are two or more source data points. If there are more than one input sources, the Combiner will select the nearest point among the input sources for every output sample.

### Parameters

The Combiner is customizable through various parameters. The parameters may be specified as private ROS parameters (read through the `rospy.get_param()` function calls) and as keyword parameters in instantiate an object of the main class `LaserscanPointcloudCombiner` in the executable program file `/nodes/laserscan_combiner.py`.

### Input ROS Topics

| Name | Default | Remarks |
| ---- | ---- | ---- |
| pointcloud_topic_name | None | A point cloud input topic (PointCloud2) |
| laserscan_in_1_topic_name | None | A laser scan input topic (LaserScan) |
| laserscan_in_2_topic_name | None | A laser scan input topic (LaserScan) |

A None value for the above input ROS topic parameters implies that the input is not used.

### Output ROS Topics

| Name | Default | Remarks |
| ---- | ---- | ---- |
| laserscan_out_topic_name | /laser_link/scan | The combined laser scan output topic (LaserScan) |
| laser_frame_id | combined_laser | The frame id of the simulated source of the laser scan output |

Note that `laser_frame_id` must have been specified in the scene (i.e., the TF tree).

### Parameters for the Input PointCloud

| Name | Default | Remarks |
| ---- | ---- | ---- |
| pointcloud_min_z | -0.2 | The lower end of the z-axis range accepted by the Combiner (in meters) in the `laser_frame_id` frame |
| pointcloud_max_z | 10.0 | The upper end of the z-axis range accepted by the Combiner (in meters) in the `laser_frame_id` frame |

### Parameters for the Output LaserScan

| Name | Default | Remarks |
| ---- | ---- | ---- |
| pub_rate | 20 | The rate of publishing the output LaserScan (Hz) |
| laser_samples | The input `LaserScan` or 120 | The number of laser scan samples of each `LaserScan` message |
| laser_min_angle | The input `LaserScan` or -2.0  | The lower end of the angle range of the output laser scan (radians) |
| laser_max_angle | The input `LaserScan` or 2.0 | The upper end of the angle range of the output laser scan (radians) |
| laser_range_min | The input `LaserScan` or 0.1 | The nearest end in the valid range of the output laser scan (meters) |
| laser_range_max | The input `LaserScan` or 20 | The furthest end in the valid range of the output laser scan (meters) |

Note that for the last five parameters, the `laserscan_in_1_topic_name` takes precedent over `laserscan_in_2_topic_name` as the default when both are specified as input sources.

### Specifying Parameters in a Launch File

A sample launch file `default.launch` is provided as an example of how to specify the parameters.

```xml
<launch>
    <node name="laserscan_combiner" pkg="mgv_laserscan_combiner" type="laserscan_combiner.py" respawn="false" output="screen">
        <param name="pointcloud_topic_name" value="/velodyne_points"/>
        <param name="laserscan_in_1_topic_name" value="/bound_detector/scan"/>
        <!-- the package supports at most 2 laserscan input and 1 pointcloud input -->
        <!-- <param name="laserscan_in_2_topic_name" value="/another_detector/scan"/> -->
        <param name="laserscan_out_topic_name" value="/laser_link/scan"/>
        <param name="laser_samples" value="240" />     
        <!-- <param name="laser_frame_id" value="velodyne" />     
        <param name="laser_min_angle" value="-3.14" />     
        <param name="laser_max_angle" value="3.14" />     
        <param name="laser_range_min" value="1" />     
        <param name="laser_range_max" value="10" />  
        <param name="pointcloud_min_z" value="-0.2" />     
        <param name="pointcloud_max_z" value="10.0" />      
        <param name="pub_rate" value="10" />      -->
    </node>
</launch>
```

The above example specifies one input `PointCloud2` source and one input `LaserScan` source. It specifies their topic names and the topic name of the output `LaserScan`. Beside the `laser_sample`, the default will be used for the other parameter values.

### Specifying Parameters in Python 

The Combiner may be executed as a ROS node through your own Python program. It is as simple as initializing the program as a ROS node and then instantiate an object of the class `LaserscanPointcloudCombiner`.

```python
    rospy.init_node(LaserscanPointcloudCombiner.NODE_NAME)
    try:
        rospy.loginfo(f'{LaserscanPointcloudCombiner.__name__}: The node "{LaserscanPointcloudCombiner.NODE_NAME}" is running (ns: {rospy.get_namespace()})')
        robot_agent = LaserscanPointcloudCombiner()
        rospy.spin()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)
```

The Combiner parameters may be specified through keywords in the call to the constructor. Note that the Combiner parameters specified as keyword parameters of the constructor can be overruled by ROS parameters.

```python
    rospy.init_node(LaserscanPointcloudCombiner.NODE_NAME)
    try:
        rospy.loginfo(f'{LaserscanPointcloudCombiner.__name__}: The node "{LaserscanPointcloudCombiner.NODE_NAME}" is running (ns: {rospy.get_namespace()})')
        robot_agent = LaserscanPointcloudCombiner(laser_samples=320, laserscan_out_topic_name=`/another_laser_link/scan`)
        rospy.spin()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)
```

## Installation

This package should be installed under the `src` folder of a workspace. Use an existing workspace or create a new one as follows.
```bash
cd ~
mkdir -p catkin_ws/src
```
Change directory to the `src` folder under the workspace.
```bash
cd catkin_ws/src
```
Clone this repository and the package is downloaded as a folder called `mgv_laserscan_combiner`.
```bash
git clone https://github.com/REF-RAS/mgv_laserscan_combiner.git
```
Build the package and launch the ROS node
```
cd ~/catkin_ws
catkin_make

roslaunch mgv_laserscan_combiner default.launch
```
The node will wait for messages from the input sources if they are not already publishing in the ROS environment.

![Figure](docs/assets/LaserScanDemo.png)


## Developer

Dr Andrew Lui, Senior Research Engineer <br />
Robotics and Autonomous Systems, Research Engineering Facility <br />
Research Infrastructure <br />
Queensland University of Technology <br />

First version: 23 Jul 2024 <br />
Latest update: 24 Jul 2024