#!/usr/bin/env python3 

# Copyright 2024 - Andrew Kwok Fai LUI, 
# Robotics and Autonomous Systems Group, REF, RI
# and the Queensland University of Technology

__author__ = 'Andrew Lui'
__copyright__ = 'Copyright 2024'
__license__ = 'GPL'
__version__ = '1.0'
__email__ = 'ak.lui@qut.edu.au'
__status__ = 'Development'

import math, random
import numpy as np
import rospy
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, TransformStamped, Vector3
from sensor_msgs.msg import LaserScan, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from tools.logging_tools import logger

class LaserscanPointcloudCombiner(): 
    NODE_NAME = 'laserscan_pointcloud_combiner'
    def __init__(self, **kwargs):
        # ros callback for shutdown
        rospy.on_shutdown(self._cb_shutdown)
        # input parameters
        self.pub_rate = rospy.get_param('~pub_rate', kwargs.get('pub_rate', 20))     # 20 Hz
        self.pointcloud_topic_name = rospy.get_param('~pointcloud_topic_name', kwargs.get('pointcloud_topic_name', None)) # '/velodyne_points'
        self.laserscan_out_topic_name = rospy.get_param('~laserscan_out_topic_name', kwargs.get('laserscan_out_topic_name', '/laser_link/scan'))
        self.laser_frame_id = rospy.get_param('~laser_frame_id', kwargs.get('laser_frame_id', 'combined_laser'))
        self.laserscan_in_1_topic_name = rospy.get_param('~laserscan_in_1_topic_name', kwargs.get('laserscan_in_1_topic_name', None))
        self.laserscan_in_2_topic_name = rospy.get_param('~laserscan_in_2_topic_name', kwargs.get('laserscan_in_2_topic_name', None))        
        # resolve the use of in_2 instead of in_1 by copying the in_2 topic to in_1 topic
        if self.laserscan_in_1_topic_name is None and self.laserscan_in_2_topic_name is not None:
            self.laserscan_in_1_topic_name = self.laserscan_in_2_topic_name
            self.laserscan_in_2_topic_name = None
        # check if there is enough input sources, which is either a pointcloud or two laserscan
        if not (self.pointcloud_topic_name is not None or (self.laserscan_in_1_topic_name is not None and self.laserscan_in_2_topic_name is not None)):
            message = f'{type(self).__name__}: Inadequate input sources, either one PointCloud2 or two LaserScan topics'
            logger.error(message)
            raise AssertionError(message)
        # model parameters
        self.lastest_in_1_laserscan = self.lastest_in_2_laserscan = self.latest_pointcloud = None
        # wait for message published by in_1 laserscan
        if self.laserscan_in_1_topic_name is not None:
            logger.info(f'{type(self).__name__}: waiting for incoming (in 1) laserscan messages from "{self.laserscan_in_1_topic_name}"')
            self.lastest_in_1_laserscan:LaserScan = rospy.wait_for_message(self.laserscan_in_1_topic_name, LaserScan)
            logger.info(f'{type(self).__name__}: discovered and was able to read a laserscan message')
        # wait for message published by in_2 laserscan 
        if self.laserscan_in_2_topic_name is not None:
            logger.info(f'{type(self).__name__}: waiting for incoming (in 2) laserscan messages from "{self.laserscan_in_2_topic_name}"')
            self.lastest_in_2_laserscan:LaserScan = rospy.wait_for_message(self.laserscan_in_2_topic_name, LaserScan)
            logger.info(f'{type(self).__name__}: discovered and was able to read a laserscan message')        
        # set input filter parameters for pointcloud z range
        self.pointcloud_min_z = rospy.get_param('~pointcloud_min_z', kwargs.get('pointcloud_min_z', -0.2))
        self.pointcloud_max_z = rospy.get_param('~pointcloud_max_z', kwargs.get('pointcloud_max_z', 10.0))
        # set default values that of the incoming laser scan if not given
        default_laser_min_angle = -2.0 if self.laserscan_in_1_topic_name is None else self.lastest_in_1_laserscan.angle_min
        default_laser_max_angle = +2.0 if self.laserscan_in_1_topic_name is None else self.lastest_in_1_laserscan.angle_max
        default_laser_sample_n = 120 if self.laserscan_in_1_topic_name is None else len(self.lastest_in_1_laserscan.ranges)
        default_laser_range_min = 0.1 if self.laserscan_in_1_topic_name is None else self.lastest_in_1_laserscan.range_min
        default_laser_range_max = 20 if self.laserscan_in_1_topic_name is None else self.lastest_in_1_laserscan.range_max       
        # set the output laserscan parameters from first the ros params and then the keyword paramters of the constructor and finally the defaults
        self.laser_min_angle = rospy.get_param('~laser_min_angle', kwargs.get('laser_min_angle', default_laser_min_angle))
        self.laser_max_angle = rospy.get_param('~laser_max_angle', kwargs.get('laser_max_angle', default_laser_max_angle))
        self.laser_sample_n = rospy.get_param('~laser_samples', kwargs.get('laser_samples', default_laser_sample_n))
        self.laser_range_min = rospy.get_param('~laser_range_min', kwargs.get('laser_range_min', default_laser_range_min))
        self.laser_range_max = rospy.get_param('~laser_range_max', kwargs.get('laser_range_max', default_laser_range_max))
        # wait for message published by the pointcloud topic
        if self.pointcloud_topic_name is not None:
            logger.info(f'{type(self).__name__}: waiting for cloudpoint messages from "{self.pointcloud_topic_name}"')
            self.latest_pointcloud:PointCloud2 = rospy.wait_for_message(self.pointcloud_topic_name, PointCloud2)
            logger.info(f'{type(self).__name__}: discovered and was able to read a cloudpoint message')
        # pre-compute the output laserscan derived parameters
        self.laser_scan_steps, self.laser_scan_angle_increment = self._compute_scan_steps(self.laser_min_angle, self.laser_max_angle, self.laser_sample_n)
        self.laser_scan_msg = self._create_laser_scan_msg()
        # create subscriber
        if self.pointcloud_topic_name is not None:
            self.pointcloud_sub = rospy.Subscriber(self.pointcloud_topic_name, PointCloud2, self._cb_pointcloud_received, queue_size=1)
        if self.laserscan_in_1_topic_name is not None:
            self.in_laserscan_1_sub = rospy.Subscriber(self.laserscan_in_1_topic_name, LaserScan, self._cb_laserscan_1_received, queue_size=1)      
        if self.laserscan_in_2_topic_name is not None:
            self.in_laserscan_2_sub = rospy.Subscriber(self.laserscan_in_2_topic_name, LaserScan, self._cb_laserscan_2_received, queue_size=1)                
        # create publisher
        self.laser_scan_pub = rospy.Publisher(self.laserscan_out_topic_name, LaserScan, queue_size=1)
        self.count = 0
        # timer callback for tf broadcast
        self.timer_broadcast = rospy.Timer(rospy.Duration(1.0/self.pub_rate), self._cb_timer)
        try:
            rospy.wait_for_message('/clock', Clock, rospy.Duration(1))
        except:
             logger.warning(f'{type(self).__name__} ({self.NODE_NAME}): The simulation clock is not running - is gazebo started?') 
        # output key information to the screen
        logger.info(f'{type(self).__name__}: publishing to {self.laserscan_out_topic_name}')
        if self.pointcloud_topic_name:
            logger.info(f'\t\t\t -> {self.pointcloud_topic_name} (PointCloud2)') 
            logger.info(f'\t\t\t\t z-range: {self.pointcloud_min_z} to {self.pointcloud_max_z} ')
        if self.laserscan_in_1_topic_name:    
            logger.info(f'\t\t\t -> {self.laserscan_in_1_topic_name} (LaserScan)')  
        if self.laserscan_in_2_topic_name: 
            logger.info(f'\t\t\t -> {self.laserscan_in_2_topic_name} (LaserScan)')     

    # computes the steps size of the output laserscan given input parameters
    def _compute_scan_steps(self, laser_min_angle, laser_max_angle, laser_sample_n) -> tuple:
        angle_step_size = (laser_max_angle - laser_min_angle) / laser_sample_n
        scan_steps = []
        for i in range(laser_sample_n):
            angle = laser_min_angle + i * angle_step_size
            scan_steps.append([angle, angle + angle_step_size])
        return scan_steps, angle_step_size
    
    # creates a LaserScan message for publishing
    def _create_laser_scan_msg(self):
        laser_scan_msg = LaserScan()
        laser_scan_msg.angle_min, laser_scan_msg.angle_max = self.laser_min_angle, self.laser_max_angle
        laser_scan_msg.angle_increment = self.laser_scan_angle_increment
        laser_scan_msg.range_min, laser_scan_msg.range_max = self.laser_range_min, self.laser_range_max
        laser_scan_msg.time_increment = laser_scan_msg.scan_time = 0.0
        laser_scan_msg.header.frame_id = self.laser_frame_id
        laser_scan_msg.intensities = np.zeros(self.laser_sample_n).tolist()
        return laser_scan_msg

    def _cb_shutdown(self):
        rospy.loginfo(f'{type(self).__name__}: the ros node is being shutdown')

    # callback when a Pointcloud2 messsage is received
    def _cb_pointcloud_received(self, msg:PointCloud2):
        self.latest_pointcloud = msg

    # callback when a LaserScan message is received on the in_1 laserscan topic
    def _cb_laserscan_1_received(self, msg:LaserScan):
        self.lastest_in_1_laserscan = msg

    # callback when a LaserScan message is received on the in_2 laserscan topic    
    def _cb_laserscan_2_received(self, msg:LaserScan):
        self.lastest_in_2_laserscan = msg

    # callback driven by the timer, which computes the output laserscan by combining the input sources
    def _cb_timer(self, event=None):
        # handles the recorded pointcloud2 message if available
        ranges:np.ndarray = np.ones(self.laser_sample_n) * np.inf 
        if self.latest_pointcloud is not None:
            data = list(pc2.read_points(self.latest_pointcloud, skip_nans=False, field_names=('x', 'y', 'z')))
            for i in range(len(data)):
                if self.pointcloud_max_z >= data[i][2] >= self.pointcloud_min_z:
                    dot = data[i][0] * 1 + data[i][1] * 0
                    mag1 = math.sqrt(math.pow(data[i][0], 2) + math.pow(data[i][1], 2))
                    mag2 = math.sqrt(math.pow(1, 2) + math.pow(0, 2))
                    beta = math.acos(dot / (mag1 * mag2)) * np.sign(data[i][1])
                    dist = math.sqrt(data[i][0] ** 2 + data[i][1] ** 2 + data[i][2] ** 2)
                    for angle_index in range(len(self.laser_scan_steps)):
                        if self.laser_scan_steps[angle_index][0] <= beta < self.laser_scan_steps[angle_index][1]:
                            ranges[angle_index] = min(ranges[angle_index], dist)
                            break
        # handles the recorded laserscan_in_1 message if available, and combines with the output computed above  
        if self.lastest_in_1_laserscan is not None:
            for index, dist in enumerate(self.lastest_in_1_laserscan.ranges):
                if np.isnan(dist):
                    continue
                angle = self.lastest_in_1_laserscan.angle_min + index * self.lastest_in_1_laserscan.angle_increment
                index_in_out_laserscan = int((angle - self.laser_min_angle) / self.laser_scan_angle_increment)
                if 0 <= index_in_out_laserscan <= len(ranges):
                    ranges[index_in_out_laserscan] = min(ranges[index_in_out_laserscan], dist)
        # handles the recorded laserscan_in_2 message if available, and combines with the output computed above
        if self.lastest_in_2_laserscan is not None:
            for index, dist in enumerate(self.lastest_in_2_laserscan.ranges):
                angle = self.lastest_in_2_laserscan.angle_min + index * self.lastest_in_2_laserscan.angle_increment
                index_in_out_laserscan = int((angle - self.laser_min_angle) / self.laser_scan_angle_increment)
                if 0 <= index_in_out_laserscan <= len(ranges):
                    ranges[index_in_out_laserscan] = min(ranges[index_in_out_laserscan], dist)
        # publish the output laserscan
        self.laser_scan_msg.header.seq = self.count
        self.laser_scan_msg.header.stamp = rospy.Time.now()  
        self.laser_scan_msg.ranges = ranges.tolist()
        self.laser_scan_pub.publish(self.laser_scan_msg)
        self.count += 1

if __name__ == '__main__':
    rospy.init_node(LaserscanPointcloudCombiner.NODE_NAME)
    try:
        rospy.loginfo(f'{LaserscanPointcloudCombiner.__name__}: The node "{LaserscanPointcloudCombiner.NODE_NAME}" is running (ns: {rospy.get_namespace()})')
        robot_agent = LaserscanPointcloudCombiner()
        rospy.spin()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)
