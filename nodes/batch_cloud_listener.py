#! /usr/bin/python
import numpy as np
import os
import time

#import roslib; roslib.load_manifest('rgbdslam')
import rospy

from sensor_msgs.msg import PointCloud2
from python_msg_conversions import pointclouds

import tf

# Dtypes
glCldType = np.dtype({ 'names':['x', 'y', 'z', 'r', 'g', 'b', 'a'],\
                     'formats':[np.float32, np.float32, np.float32, np.float32,\
                               np.float32, np.float32, np.float32]})     
rosCldType = np.dtype({ 'names':['x', 'y', 'z', 'r', 'g', 'b'],\
                     'formats':[np.float32, np.float32, np.float32, np.uint8,\
                               np.uint8, np.uint8]})     

def convertToRT(trans, rot):
  '''Convert the translation vector and quaternion rotation that come from
     tf by default to a 3x4 RT matrix'''
  RT = np.zeros((3,4))
  RT[0:3, 0:3] = tf.transformations.quaternion_matrix(rot)[0:3, 0:3]
  RT[:,-1] = trans
  return RT

class BatchCloudBuffer(object):
  '''The BatchCloudListener instantiates a rosnode that listens on the 
     batch_clouds topic. Both optimized transforms and point clouds are
     delivered to this node.'''
  def __init__(self, verbose=True):
    self.tf_listener = tf.TransformListener()
    self.subscriber = rospy.Subscriber("rgbdslam/batch_clouds", PointCloud2,\
                                       self.callback)
    self.cloud_publisher = rospy.Publisher("buffered_clouds", PointCloud2)#,\
#		                           queue_size=10)
    self.ctr = 0
    self.verbose = verbose
    # Accumulator
    self.buffer_size = 10
    self.resetBuffer()

  def callback(self, cloud_msg):
    tic = time.time()
    # Increment counter
    self.ctr += 1
    if self.verbose: print 'Cloud %i received' %self.ctr
    # Get the cloud timestamp
    ts = cloud_msg.header.stamp
    # Use ts to look up optimized transform and get into RT format
    try:
      (trans, rot) = self.tf_listener.lookupTransformFull("/map", ts,\
                     "/camera_rgb_optical_frame", ts, "/map")
      RT = convertToRT(trans, rot)
      # Get the point cloud
      cloud_arr = pointclouds.pointcloud2_to_array(cloud_msg, split_rgb=True)
      # Throw out points where there is no depth data
      cloud_arr = cloud_arr[np.isfinite(cloud_arr['z'])]
      # Get the spatial points only and apply transformation
      xyz = np.squeeze(np.array([cloud_arr['x'], cloud_arr['y'], cloud_arr['z'],\
                                 np.ones(cloud_arr.shape[-1])]))
      xyz = RT.dot(xyz).T
      # Get colors
      clrs = np.squeeze(np.array([cloud_arr['r'], cloud_arr['g'], cloud_arr['b']])).T
      # Concatenate data
      self.xyz = np.concatenate((self.xyz, xyz))
      self.rgb = np.concatenate((self.rgb, clrs))
      toc = time.time()
      if self.verbose: print '%.5f sec to process cloud %i' %((toc-tic), self.ctr)
    except tf.ExtrapolationException:
	print 'Floating point error'
    # If buffered data to be sent, send it
    if self.ctr % self.buffer_size == 0:
      self.sendBufferedClouds()

  def sendBufferedClouds(self):
    # Convert cloud to proper format
    tic = time.time()
    cloud = self.convertToCloudArray(rosCldType)
    cloud = cloud.view(np.recarray)
    cloud_msg = pointclouds.array_to_pointcloud2(cloud)
    # Send cloud
    self.cloud_publisher.publish(cloud_msg)
    toc = time.time()
    # Reset cloud buffer
    self.resetBuffer()
    if self.verbose:
      print
      print 'Buffered clouds sent! %.5f sec to send clouds.' %(toc-tic)
      print

  def resetBuffer(self):
    '''Reset the 3D point and color buffers'''
    self.xyz = np.array([[0, 0, 0]], dtype=np.float32)
    self.rgb = np.array([[0, 0, 0]], dtype=np.uint8)

  def convertToCloudArray(self, cldType):
    '''Convert cloud from internal processing format to format used for 
       other sending/saving tasks'''
    # Set the cloud type (gl or ros)
    outcld = np.zeros(self.xyz.shape[0], dtype=cldType)
    # xyz points are the same for both formats
    outcld['x'] = self.xyz[:,0]
    outcld['y'] = self.xyz[:,1]
    outcld['z'] = self.xyz[:,2]
    # For saving/plotting with opengl
    if cldType is glCldType:
      outcld['r'] = self.rgb[:,0]/255.
      outcld['g'] = self.rgb[:,1]/255.
      outcld['b'] = self.rgb[:,2]/255.
      outcld['a'] = np.ones(len(self.rgb))
    # For converting back into PointCloud2
    if cldType is rosCldType:
      outcld['r'] = self.rgb[:,0]
      outcld['g'] = self.rgb[:,1]
      outcld['b'] = self.rgb[:,2]
    return outcld

  def saveModel(self, blerg):                                                 
    print 'Saving model...'
    outcld = self.convertToCloudArray(glCldType)
    np.save('/home/grim5/Desktop/mergedBatchClouds.npy', outcld)
    print 'Model saved.'

if __name__ == '__main__':
  rospy.init_node('batch_cloud_listener', anonymous=True)
  listener = BatchCloudBuffer()
  rospy.spin()
  rospy.on_shutdown(listener.saveModel)
