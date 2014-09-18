#! /usr/bin/python
import numpy as np
import os
import sys
import time

import rospy

from dtype_defines import edataType

# Set up python environment for using PythonFramework tools
PF_DIR = os.path.expanduser("~") + "/PythonFramework/"
sys.path.append(PF_DIR)

# Framework imports
from src.SIS import sis
from src.cythoned import getEventPtrs
from src.analysisHelperFunctions import interactionType
from src.CAnalysis.canalysis import analyzeReadoutData

class SISAcquisitionAndAnalysisNode(object):
  '''The SISAcquisitionAndAnalysisNode will insantiate a node that is 
     responsible for acquiring data from a SIS system, '''
  def __init__(self, verbose=True, read_rate=0.5):
    # Node attributes
    self.ctr = 0
    self.msg = None
    self.edata = None
    self.stamp = None
    self.data_out = None
    self.rate = rospy.rate(read_rate)	# Hz, default is every 2 seconds
    self.iary = np.zeros(1, dtype=interactionType) # For CAnalysis
    # SIS flags - NOTE: HARD-CODED FOR NOW, SHOULD MAKE CONFIGURABLE
    self.save_data = 0	# Default: Don't save binary data
    self.cal_file = PF_DIR + 'src/SIS/CCI2_Calibration_4-9-13.txt'
    self.conf_file = PF_DIR +\
    'src/SIS/DATA_nnOn_trig=50=21.5keV_optimizedFilter_PostPreampRepair.ini'
    self.binary_file = os.path.expanduser("~") + '/Desktop/ROS_DET_NODE_OUT.dat'
    # Connect to SIS hardware
    sis.connectToDAQ()
    sis.configuration(self.conf_file, self.binary_file, self.save_data)
    self.hardware_started = False

  def run(self)
    while not rospy.is_shutdown():
      # Do task
      self.task()
      # Log stuff
      rospy.loginfo(self.msg)
      # Publish
      if self.data_out is not None:
        self.publisher.publish(self.data_out)
      # Wait for next dump
      self.rate.sleep()

  def task(self):
    if self.hardware_started:
      # Data acquisition and event reconstruction goes here
      pass
    else:
      msg = '[SIS_NODE]: Hardware not started, awaiting signal...'

  def acquire_data(self):
    # Acquire data from sis
    tic = time.time()
    ts, en, ch, trig = sis.acquiredata(self.cal_file, self.save_data)
    acq_ts = time.time()	# Acquisition time stamp
    # Convert to numpy array
    edata = np.zeros(ts.shape, dtype=edataType)
    edata = edata.view(np.recarray)
    edata.timestamp = ts
    edata.ADC_value = en
    edata.detector = ch
    edata.trigger = trig
    # Set up data for further processing
    self.edata = edata
    self.stamp = acq_ts
    self.ctr += 1
    toc = time.time()
    # Add logging/printing here

  def preprocess_data(self):
    # After having acquired new data, preprocess it to get it ready for
    # reconstruction
    #NOTE: Filters are all hard-coded here
    if self.edata is None: return
    # Trigger filter (reject NN and pileup)
    self.edata = self.edata[self.edata['trigger'] == 1]
    # LLD
    self.edata = self.edata[self.edata['ADC_value'] >= 25]
    # Reject GR signals
    self.edata = self.edata[self.edata['detector'] % 38 != 0]
    # Sort
    self.edata.sort(order='timestamp')

  def reconstruct_compton_events(self):
    # Take the preprocessed data and reconstruct all the 2-interaction events
    if self.edata is None: return
    # Get time-correlated readouts - NOTE: Only looking for 2-int events!
    ptrs, lens = getEventPtrs(self.edata['timestamp'], 40, 4, 4)
    l11, l22, l33, lerr = analyzeReadoutData(self.edata, ptrs, lens, self.iary)
    # Set the data to be sent out
    self.data_out = l22

  def start_hardware(self):
    # Start the acquisition from SIS
    sis.startacquisition()
    self.start_time = time.time()
    self.hardware_started = True

if __name__ == '__main__':
  rospy.init_node('detector_publisher', anonymous=True)
  detector_publisher = SISAcquisitionAndAnalysisNode()
  rospy.spin()
#  rospy.on_shutdown(listener.saveModel)
