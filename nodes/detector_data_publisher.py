#! /usr/bin/python
import numpy as np
import os
import sys
import time
import tables

import rospy

from dtype_defines import edataType

# Set up python environment for using PythonFramework tools
PF_DIR = os.path.expanduser("~") + "/PythonFramework/"
sys.path.append(PF_DIR)

# Framework imports
from src.SIS import sis
from src.cythoned.getPtrs_cy import getEventPtrs
from src.analysisHelperFunctions import interactionType, et50_type,\
		                        convert_to_t50_type, correct_depth,\
					refine_z
#from src.CAnalysis.canalysis import analyzeReadoutData
from src.analysis_v2 import analyzeReadoutData

savedInteractionType = np.dtype({"names":['time','energy1', 'x1', 'y1', 'z1', 'dT1',\
                                         'dE1', 'det1', 'energy2', 'x2', 'y2', 'z2',\
                                         'dT2', 'dE2', 'det2'],                 
                                 "formats":[np.float64, np.float32, np.float32,\
                                            np.float32, np.float32, np.float32,\
                                            np.float32, np.float32, np.float32,\
                                            np.float32, np.float32, np.float32,\
                                            np.float32, np.float32, np.float32]} )

from ZmqClass import ZmqEmitter

class SISAcquisitionAndAnalysisNode(object):
  '''The SISAcquisitionAndAnalysisNode will insantiate a node that is 
     responsible for acquiring data from a SIS system, '''
  def __init__(self, verbose=True, read_rate=2):
    # Node attributes
    self.ctr = 0
    self.msg = None
    self.edata = None
    self.data_out = np.empty(0, dtype=savedInteractionType)
    self.rate = rospy.Rate(read_rate)	# Hz, default is every 2 seconds
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
    # Set up ZMQ publisher
    self.publisher = ZmqEmitter('5556', 'interactions')

  def run(self):
    while not rospy.is_shutdown():
      # NOTE: Want this to be controlled via input - hard-coded for testing
      if not self.hardware_started:
	self.start_hardware()
	self.hardware_started = True
	continue
      # Wait for next dump
      self.rate.sleep()
      # Do task
      self.task()
      # Log stuff
#      rospy.loginfo(self.msg)

  def task(self):
    if self.hardware_started:
      ts = self.acquire_data()
      self.preprocess_data()
      l22 = self.reconstruct_compton_events()
      print l22.shape
      num_orig = l22.shape[0]
      # Format data for sending via ros
      if len(l22) > 0: self.send_interactions(l22, ts)
    else:
      msg = '[SIS_NODE]: Hardware not started, awaiting signal...'

  def acquire_data(self):
    # Acquire data from sis
    tic = time.time()
    ts, en, ch, trig, rdata = sis.acquireDataWithRaw(self.cal_file, self.save_data)
    acq_ts = rospy.get_time()
    # Convert to numpy array
    edata = np.zeros(ts.shape, dtype=edataType)
    edata = edata.view(np.recarray)
    edata.timestamp = ts
    edata.ADC_value = en
    edata.detector = ch
    edata.trigger = trig
    # Set up data for further processing
    self.edata = edata
    self.rdata = rdata
    self.ctr += 1
    toc = time.time()
    # Add logging/printing here

    return acq_ts

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
    # Extract t50
    self.edata = convert_to_t50_type(self.edata, self.rdata)

  def reconstruct_compton_events(self):
    # Take the preprocessed data and reconstruct all the 2-interaction events
    if self.edata is None: return
    # Get time-correlated readouts - NOTE: Only looking for 2-int events!
    ptrs, lens = getEventPtrs(self.edata['timestamp'], 40, 4, 4)
    l11, l22, l33, lerr = analyzeReadoutData(self.edata, ptrs, lens, self.iary)
    # Remove bad dt50 values
    l22['dt50'][np.invert(np.isfinite(l22['dt50']))] = 0.0                      
    # Refine z
    l22['z'] = refine_z(l22)                                                    
    # Set the data to be sent out
    return l22

  def start_hardware(self):
    # Start the acquisition from SIS
    sis.startacquisition()
    self.start_time = time.time()
    self.hardware_started = True

  def send_interactions(self, inter22, acq_ts):
    # Convert to proper format
    numEvents = len(inter22)
    newEvs = np.zeros(numEvents, dtype=savedInteractionType)
    for i in xrange(numEvents): 
      int1 = inter22[i][0] 
      int2 = inter22[i][1]
      newEvs[i]['time'] = acq_ts 
      newEvs[i]['energy1'] = int1[0]
      newEvs[i]['energy2'] = int2[0]
      newEvs[i]['x1'] = int1[1] 
      newEvs[i]['x2'] = int2[1] 
      newEvs[i]['y1'] = int1[2] 
      newEvs[i]['y2'] = int2[2] 
      newEvs[i]['z1'] = int1[3]
      newEvs[i]['z2'] = int2[3]
      newEvs[i]['dT1'] = int1[4]
      newEvs[i]['dT2'] = int2[4]
      newEvs[i]['dE1'] = int1[5]
      newEvs[i]['dE2'] = int2[5]
      newEvs[i]['det1'] = int1[6]
      newEvs[i]['det2'] = int2[6]
    # Append new interactions to the existing ones
    self.data_out = np.concatenate((self.data_out, newEvs)) 
    # Send all interactions
    self.publisher.send_zipped_pickle(self.data_out)

  def save_interactions(self):
    hdr = '#time\te1\tx1\ty1\tz1\tdT1\tdE1\tdet1\te2\tx2\ty2\tz2\tdT2\tdE2\tdet2\n'
    np.savetxt('/home/grim5/Desktop/ints22.txt', self.data_out, header=hdr)

if __name__ == '__main__':
  rospy.init_node('detector_publisher', anonymous=True)
  detector_publisher = SISAcquisitionAndAnalysisNode()
  raw_input('Press Enter to begin detector acquisition: ')
  print 'Starting detector acquisition...'
  detector_publisher.run()
#  detector_publisher.save_interactions()
