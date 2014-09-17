#! /usr/bin/python
import numpy as np
import os
import time

import rospy

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped
from python_msg_conversions.pointclouds import pointcloud2_to_xyz_array

import tf

from dtype_defines import stamped_transform_type, xyz_cloud_type
from converter_functions import convertToRT, convertQuaternionToTuple,\
		                convertTranslationToTuple

TRANSFORM_BUFFER_SIZE = 10000

class DataAggregator(object):
  '''The DataAggregator is designed to collect data from all other sources. The
     node should handle buffered point clouds, time-stamped transformations, and
     gamma-ray data (produced by a SISDAQ/Analysis node). Once the aggregation
     if figured out, the node can be hooked up with the volumetric imaging
     code.'''
  def __init__(self, verbose=True):
    # Node attributes
    self.cloud_ctr = 0
    self.transform_ctr = 0
    # Set up listeners for the cloud and transforms
    self.cloud_listener = rospy.Subscriber("buffered_clouds", PointCloud2,\
		                           self.cloud_callback)
    self.transform_listener = rospy.Subscriber("unbuffered_poses",\
		                               TransformStamped,\
					       self.transform_callback)
    self.verbose = verbose
    # Accumulators
    self.cloud = np.empty(0, dtype=xyz_cloud_type)
    self.transforms = np.zeros(TRANSFORM_BUFFER_SIZE,\
		               dtype=stamped_transform_type)

  def cloud_callback(self, cloud_msg):
    '''Get the latest cloud dump. Extract xyz positions and add to self.cloud'''
    tic = time.time()
    self.cloud_ctr += 1
    xyz = pointcloud2_to_xyz_array(cloud_msg, remove_nans=False)
    self.cloud = np.concatenate(self.cloud, xyz)
    toc = time.time()
    if self.verbose:
      print
      print 'Cloud packet received: %s points, %.5f sec to update'\
	    %(len(xyz), (toc-tic))
      print

  def transform_callback(self, tfmsg):
    '''Get the latest transform, parse, and add it to the node transform list'''
    tic = time.time()
    # Parse message
    ts = tfmsg.header.stamp.to_nsec()
    trans = convertTranslationToTuple(tfmsg.transform.translation)
    rot = convertQuaternionToTuple(tfmsg.transform.rotation)
    # Convert to proper format
    RT = convertToRT(trans, rot)
    # Add to self.transforms
    self.transforms['timestamp'][self.transform_ctr] = ts
    self.transforms['RT'][self.transform_ctr] = RT
    # Increment
    self.transform_ctr += 1
    toc = time.time()
    if self.verbose:
      print 'Transform received, %.5f to process' %(toc-tic)

if __name__ == '__main__':
  rospy.init_node('data_aggregator', anonymous=True)
  listener = DataAggregator()
  rospy.spin()
#  rospy.on_shutdown(listener.saveModel)
