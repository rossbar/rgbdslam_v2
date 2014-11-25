#! /usr/bin/python
import numpy as np
import os
import time

import rospy

class SISAcquisitionAndAnalysisNode(object):
  '''The SISAcquisitionAndAnalysisNode will insantiate a node that is 
     responsible for acquiring data from a SIS system, '''
  def __init__(self, verbose=True, read_rate=0.5):
    # Node attributes
    self.ctr = 0
    self.rate = rospy.rate(read_rate)	# Hz, default is every 2 seconds

  def run(self)
    pass

if __name__ == '__main__':
  rospy.init_node('detector_publisher', anonymous=True)
  detector_publisher = SISAcquisitionAndAnalysisNode()
  rospy.spin()
#  rospy.on_shutdown(listener.saveModel)
