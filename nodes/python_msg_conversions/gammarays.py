import numpy as np
from rgbdslam.msg import GammaRayInteraction, ComptonEvent, ComptonEventList

from dtype_defines import interactionType

def numpy_to_ComptonEvent(ev, ts=None, sequenced=None):
  '''Convert a numpy array of 2-site compton event (row 0 = first interaction, 
     row 1 = second interaction) into a ComptonEvent rosmsg'''
  msg = ComptonEvent()
  if ts is not None: msg.header.stamp.nsecs = ts
  if sequenced is not None: msg.is_sequenced = sequenced
  msg.int1 = GammaRayInteraction(*ev[0])
  msg.int2 = GammaRayInteraction(*ev[1])
  return msg

def ComptonEvent_to_numpy(cev):
  i1 = cev.int1
  i2 = cev.int2
  ev = np.zeros(2, dtype=interactionType)
  ev[0] = (i1.energy, i1.x, i1.y, i1.z, i1.dT, i1.dE, i1.detector)
  ev[1] = (i2.energy, i2.x, i2.y, i2.z, i2.dT, i2.dE, i2.detector)
  return ev

def numpy_to_ComptonEventList(l22, ts=None):
  '''Convert a list of 2-site compton events (e.g. l22 as produced by 
     analyzeReadoutData after it has been reshaped) into a ComptonEventList
     ros msg'''
  print 'Creating message...'
  msg = ComptonEventList()
  print 'Assigning timestamp...'
  if ts is not None: msg.header.stamp.nsecs = ts
  # Tag the number of events
  msg.num_events = len(l22)
  print 'Assigning number of events...'
  # Turn each event in l22 into a message and insert into 
  print 'Assigning events to list...'
  ce_list = [numpy_to_ComptonEvent(ev) for ev in l22]
  msg.events = ce_list
  print 'Done!'
  return msg

def ComptonEventList_to_numpy(event_list):
  # Set up
  ts = event_list.header.stamp.to_nsec()
  events = event_list.events
  l22 = np.zeros((event_list.num_events, 2), dtype=interactionType)
  # Convert
  for i, ev in enumerate(events):
    l22[i] = ComptonEvent_to_numpy(ev)
  return ts, l22
