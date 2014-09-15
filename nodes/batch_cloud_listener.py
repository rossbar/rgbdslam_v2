#! /usr/bin/python
import numpy as np
import os
import time

#import roslib; roslib.load_manifest('rgbdslam')
import rospy

from sensor_msgs.msg import PointCloud2
from python_msg_conversions import pointclouds

import tf


def convertToRT(trans, rot):
  '''Convert the translation vector and quaternion rotation that come from
     tf by default to a 3x4 RT matrix'''
  RT = np.zeros((3,4))
  RT[0:3, 0:3] = tf.transformations.quaternion_matrix(rot)[0:3, 0:3]
  RT[:,-1] = trans
  return RT

class BasicListener():
  '''The BasicListener instantiates a rosnode that listens on the 
     batch_clouds topic. The node print out every time a new cloud_msg is
     received.'''
  def __init__(self):
    self.tf_listener = tf.TransformListener()
    self.subscriber = rospy.Subscriber("rgbdslam/batch_clouds", PointCloud2,\
                                       self.callback)
    self.ctr = 0

  def callback(self, cloud_msg):
    '''Print out that a cloud message was received.'''
    self.ctr += 1
    print "Cloud #%s received!" %(self.ctr)

class BatchCloudListener(object):
  '''The BatchCloudListener instantiates a rosnode that listens on the 
     batch_clouds topic. Both optimized transforms and point clouds are
     delivered to this node.'''
  def __init__(self):
    self.tf_listener = tf.TransformListener()
    self.subscriber = rospy.Subscriber("rgbdslam/batch_clouds", PointCloud2,\
                                       self.callback)
    self.ctr = 0

  def callback(self, cloud_msg):
    # Increment counter
    self.ctr += 1
    # Get the cloud timestamp
    ts = cloud_msg.header.stamp
    # Use ts to look up optimized transform and get into RT format
    (trans, rot) = self.tf_listener.lookupTransformFull("/camera_link", ts,\
                                                        "/map", ts, "/map")
    RT = convertToRT(trans, rot)
    # Get the point cloud
    cloud_arr = pointclouds.pointcloud2_to_array(cloud_msg, split_rgb=True)
    # Throw out points where there is no depth data
    cloud_arr = cloud_arr[np.isfinite(cloud_arr['z'])]
    # Get the spatial points only and apply transformation
    xyz = np.squeeze(np.array([cloud_arr['x'], cloud_arr['y'], cloud_arr['z'],\
                               np.ones(cloud_arr.shape[-1])]))
#    print 'RT shape = ', RT.shape
#    print 'xyz shape = ', xyz.shape
    xyz = np.dot(RT, xyz)
#    print 'new xyz shape = ', xyz.shape
    # 

    


if __name__ == '__main__':
  rospy.init_node('batch_cloud_listener', anonymous=True)
  listener = BatchCloudListener()
  rospy.spin()
