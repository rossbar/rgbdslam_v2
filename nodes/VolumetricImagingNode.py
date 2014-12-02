#! /usr/bin/python

import numpy as np
import os
import time

import rospy

# Messages
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped
from rgbdslam.msg import ComptonEventList
# Message conversion functions
from python_msg_conversions import pointcloud2_to_xyz_array
from python_msg_conversions.gammarays import ComptonEventList_to_numpy as ce2np

import tf

from dtype_defines import stamped_transform_type, xyz_cloud_type, edataType
from converter_functions import convertToRT, convertQuaternionToTuple,\
                                convertTranslationToTuple

TRANSFORM_BUFFER_SIZE = 10000

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
  def cloud_callback(self, cloud_msg):
    '''Append xyz positions to point cloud'''
    self.cloud_counter += 1
    xyz = pointcloud2_to_xyz_array(cloud_msg, remove_nans=False)
    self.cloud = np.concatenate(self.cloud, xyz)

  def transform_callback(self, tfmsg):
    '''Append new transformations'''
    # Parse message
    ts = tfmsg.header.stamp.to_nsec()
    trans = convertTranslationToTuple(tfmsg.transform.translation)
    rot = convertQuaternionToTuple(tfmsg.transform.rotation)
    RT = convertToRT(trans, rot)
    # Append
    self.transforms['timestamp'][self.pose_counter] = ts
    self.transforms['RT'][self.pose_counter] = RT
    self.pose_counter += 1

  def compton_callback(self, event_list):
    '''Append new Compton data from the detector'''
    ts, l22 = ce2np(event_list)

