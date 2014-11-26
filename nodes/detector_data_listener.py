#! /usr/bin/python
import numpy as np
import os
import sys
import time

import rospy

from dtype_defines import edataType
from rgbdslam.msg import ComptonEventList
from python_msg_conversions.gammarays import ComptonEventList_to_numpy as ce2np

# Set up python environment for using PythonFramework tools
PF_DIR = os.path.expanduser("~") + "/PythonFramework/"
sys.path.append(PF_DIR)

# Framework imports
from src.SIS import sis
from src.analysisHelperFunctions import interactionType
from src.CAnalysis.canalysis import analyzeReadoutData

class BasicDetectorListenerNode(object):
  '''Listens for compton events on the /compton_events topic and prints them
     when it gets them. This node is just for testing that everything is
     working okay.'''
  def __init__(self, verbose=True, read_rate=0.5):
    # Node attributes
    self.ctr = 0
    self.events = np.empty((0, 2), dtype=interactionType, order='F')
    self.stamps = np.empty(0, dtype=np.float)

    # Subscriber
    self.subscriber = rospy.Subscriber("/compton_events", ComptonEventList,\
		                       self.callback)

  def callback(self, event_list):
    '''Parsed the received ComptonEventList message back into a num_events x 2
       numpy array where the first column is the first interaction etc.'''
    ts, l22 = ce2np(event_list)
    ts = np.array(ts, ndmin=1)
    print 'Got %s compton events with timestamp %s' %(l22.shape[0], ts)
    print l22.shape
    self.stamps = np.concatenate((self.stamps, ts), axis=1)
    self.events = np.concatenate((self.events, l22))

  def save_events(self, rosblah):
    print 'rosblah = ', rosblah
    np.savetxt('/home/grim5/Desktop/ts_from_detector_listener.txt', self.stamps)
    np.savetxt('/home/grim5/Desktop/l22_from_detector_listener.txt',\
	       self.events.ravel())

if __name__ == '__main__':
  rospy.init_node('detector_listener', anonymous=True)
  detector_listener = BasicDetectorListenerNode()
  rospy.spin()
  rospy.on_shutdown(detector_listener.save_events)
