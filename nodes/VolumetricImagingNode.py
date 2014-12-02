#! /usr/bin/python

import numpy as np
import os
import time

### ROS imports
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
### /ROS imports

### Volumetric imports
import comptonMLEM3D
from positionInteractionsByTiming import positionInteractionsByTiming

TRANSFORM_BUFFER_SIZE = 10000

class VolumetricImagingNode(object):
  '''This node listens on 3 topics: /unbuffered_poses and /buffered_clouds
     (from BatchCloudBuffer) and /compton_events (from detector acquisition
     node). Every time a new batch of detector data is sent, recompute the 
     3D radiation image using Andy's MLEM algorithm. Publish result on a new
     topic that will link back up to the GUI.'''
  def __init__(self, verbose=True):
    # Imagine parameters
    self.energy = 662.
    self.use_sdf = True	# Use cloud to restrict image space
    self.n_iters = 10	# Number of EM iterations
    self.sequence_events = True	# Toggle whether to use event sequencing
    # Keep track of number of data received
    self.pose_counter = 0
    self.cloud_counter = 0
    self.detector_counter = 0
    self.imaging_counter = 0
    # Arrays for storing data
    self.cloud = np.empty(0, dtype=xyz_cloud_type)
    self.transforms = np.zeros(TRANSFORM_BUFFER_SIZE,\
		               dtype=stamped_transform_type)
    self.events = np.empty((0, 2), dtype=interactionType)
    self.event_stamps = np.empty(0, dtype=np.float)
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
    # Event sequencing #TODO!!!!
    if self.sequence_events:
      pass
    ts = np.array(ts, ndmin=1)
    self.event_stamps = np.concatenate((self.event_stamps, ts), axis=1)
    self.events = np.concatenate((self.events, l22))

  ### Imaging
  def update_radiation_image(self):
    '''Take the cloud, pose, and event buffers stored in self and using the
       3D MLEM algorithm to compute a radiation image.'''
    # Convert interactions to 3D interactions
    # NOTE: This will take some kajiggering to get the data in the right formats
    interactions_3D = positionInteractionsByTiming(self.events,\
		      self.transforms, makePlots=False, E=662.)
    cIm = comptonMLEM3D.comptonMLEM3D(dataSource=interactions_3D, sigTheta=3.,\
		                      pixSize=0.1, rangeMult=1)
    if self.use_sdf:
      mask_array = cIm.setGridFromCld(self.cloud, 0.1)
    # Compute
    cIm.computMLEMsphere(self.n_iters, showPlots=False, verbose=False,\
		         nSph=10000, plotSphere=False, maskArray=maskArray)
