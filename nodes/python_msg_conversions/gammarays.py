import numpy as np
from rgbdslam.msg import GammaRayInteraction, ComptonEvent

def numpy_to_ComptonEvent(ev, ts=None, sequenced=None):
  '''Convert a numpy array of 2-site compton event (row 0 = first interaction, 
     row 1 = second interaction) into a ComptonEvent rosmsg'''
  msg = ComptonEvent()
  if ts is not None: msg.header.stamp = ts
  if sequenced is not None: msg.is_sequenced = sequenced
  msg.int1 = GammaRayInteraction(*ev[0])
  msg.int2 = GammaRayInteraction(*ev[1])
  return msg
