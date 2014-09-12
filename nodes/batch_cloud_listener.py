#! /usr/bin/python
import numpy as np
import os
import time

#import roslib; roslib.load_manifest('rgbdslam')
import rospy

from sensor_msgs.msg import PointCloud2
from python_msg_conversions import pointclouds

import tf

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


if __name__ == '__main__':
  rospy.init_node('batch_cloud_listener', anonymous=True)
  listener = BasicListener()
  rospy.spin()
