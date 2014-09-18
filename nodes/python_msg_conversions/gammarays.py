import numpy as np
from rgbdslam.msg import GammaRayInteraction, ComptonEvent, ComptonEventList

def numpy_to_ComptonEvent(ev, ts=None, sequenced=None):
  '''Convert a numpy array of 2-site compton event (row 0 = first interaction, 
     row 1 = second interaction) into a ComptonEvent rosmsg'''
  msg = ComptonEvent()
  if ts is not None: msg.header.stamp = ts
  if sequenced is not None: msg.is_sequenced = sequenced
  msg.int1 = GammaRayInteraction(*ev[0])
  msg.int2 = GammaRayInteraction(*ev[1])
  return msg

def numpy_to_ComptonEventList(l22, ts=None):
  '''Convert a list of 2-site compton events (e.g. l22 as produced by 
     analyzeReadoutData after it has been reshaped) into a ComptonEventList
     ros msg'''
  msg = ComptonEventList()
  if ts is not None: msg.header.stamp = ts
  # Tag the number of events
  msg.num_events = len(l22)
  # Turn each event in l22 into a message and insert into 
  ce_list = [numpy_to_ComptonEvent(ev) for ev in l22]
  msg.events = ce_list
  return msg
