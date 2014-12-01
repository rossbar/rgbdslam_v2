#! /usr/bin/python

import numpy as np
import os
import time

import rospy

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped
from python_msg_conversions import pointclouds

#ADD DTYPES

import tf

class VolumetricImagingNode(object):
  '''This node listens on 3 topics: /unbuffered_poses and /buffered_clouds
     (from BatchCloudBuffer) and /compton_events (from detector acquisition
     node). Every time a new batch of detector data is sent, recompute the 
     3D radiation image using Andy's MLEM algorithm. Publish result on a new
     topic that will link back up to the GUI.'''
  def __init__(self, verbose=True):
    # Keep track of number of data received
    self.pose_counter = 0
    self.cloud_counter = 0
    self.detector_counter = 0
    self.imaging_counter = 0
    # Arrays for storing data
    self.cloud = np.empty(0, dtype=xyz_cloud_type)
    self.transforms = np.zeros(TRANSFORM_BUFFER_SIZE,\
		               dtype=stamped_transform_type)
    # Set up listeners
    self.pose_listener = rospy.Subscriber("unbuffered_poses", TransformStamped,\
		                          self.transform_callback)
    self.cloud_listener = rospy.Subscriber("buffered_clouds", PointCloud2,\
		                           self.cloud_callback)
    self.compton_listener = rospy.Subscriber("compton_events",\
		                             ComptonEventList,\
					     self.compton_callback)

  #### Callbacks for appending data
