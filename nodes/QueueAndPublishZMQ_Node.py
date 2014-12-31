#! /usr/bin/python                                                              
import numpy as np                                                              
import os                                                                       
import time                                                                     
                                                                                
import rospy                                                                    
                                                                                
from sensor_msgs.msg import PointCloud2                                         
from geometry_msgs.msg import TransformStamped                                  
from python_msg_conversions import pointclouds                                  
                                                                                
import tf                                                                       
                                                                                
from dtype_defines import glCldType, rosCldType                                 
from converter_functions import convertToRT, convertToStampedTransform

ttRType = np.dtype( {"names":['timestamp', 'tx', 'ty', 'tz', 'ROO', 'R01',\
                             'R02', 'R10', 'R11', 'R12', 'R20', 'R21', 'R22'],
                     "formats":[np.float64,np.float32,np.float32,np.float32,\
                                np.float32,np.float32,np.float32,np.float32,\
                                np.float32,np.float32,np.float32,np.float32,\
                                np.float32]} )

from ZmqClass import ZmqEmitter

class CloudAndPoseBuffer_ZMQPublisher(object):
  '''Create a buffer for poses and point clouds (individual frames). Publish 
     the results on zmq topics at user-defined intervals (queue size or time)'''
  def __init__(self, tf_port="5557", pc_port="5558", tf_topic="poses",\
	       pc_topic="pointCloud"):
    # Create objects to listen on ros topics
    self.tf_listener = tf.TransformListener()
    self.pc_listener = rospy.Subscriber("rgbdslam/batch_clouds", PointCloud2,\
                                        self.new_pc_callback)
    # Buffers for cloud data and pose data
    self.point_cloud = np.empty(0, dtype=rosCldType)
    self.poses = np.empty(0, dtype=ttRType)
    # ZMQ Publishers
    self.tf_emitter = ZmqEmitter(tf_port, tf_topic)
    self.pc_emitter = ZmqEmitter(pc_port, pc_topic)

    # Count how many poses and point clouds have been received
    self.ctr = 0
    # Publication interval
    self.num_to_send = 10
  
  def new_pc_callback(self, cloud_msg):
    '''When a new point cloud is received over the batch_clouds topic, 
       look up the corresponding transform, apply it to the point cloud, and
       store both in their corresponding buffers.'''
    tic = time.time()
    self.ctr += 1
    # Retrieve cloud timestamp
    ts = cloud_msg.header.stamp
    # Use timestamp to look up corresponding transformation. Catch
    # extrapolation errors and notify user
    try:
      (trans, rot) = self.tf_listener.lookupTransformFull("/map", ts,\
                     "/camera_rgb_optical_fram", ts, "/map")
      R = rot[0:3, 0:3]
    except tf.ExtrapolationException:
      print 'WARNING: FLOATING POINT ERROR IN tf.lookupTransformFull'
    # Add pose to buffer
    ts_nsec = ts.nsecs
    new_pose = np.array([(ts_nsec,) + trans + tuple(R.flatten())],\
                        dtype=ttRType)
    self.poses = np.concatenate((self.poses, new_pose))
    # Convert pose to RT matrix for application to point cloud
    RT = convertToRT(trans, rot)
    # Get point cloud
    cloud_arr = pointclouds.pointcloud2_to_array(cloud_msg, split_rgb=True)
    cloud_arr = cloud_arr[np.isfinite(cloud_arr['z'])]
    xyz = np.squeeze(np.array([cloud_arr['x'], cloud_arr['y'], cloud_arr['z'],\
                               np.ones(cloud_arr.shape[-1])]))
    # Apply pose to pc
    xyz = RT.dot(xyz).T
    cloud_arr['x'] = xyz[:,0]
    cloud_arr['y'] = xyz[:,1]
    cloud_arr['z'] = xyz[:,2]
    # Add new pc data to cloud buffer
    np.concatenate((self.point_cloud, cloud_arr))
    toc = time.time()
    
    # If publication criterion met, send data out viz zmq
    if self.ctr % self.num_to_send == 0:
      self.tf_emitter.send_zipped_pickle(self.poses)
      self.pc_emitter.send_zipped_pickle(self.point_cloud)
      # Reset pc buffer
      self.point_cloud = np.empty(0, dtype=rosCldType)
