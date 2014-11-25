import numpy as np
import tf
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion, Transform

# Given a translation and rotation in the ros format (3x1 vector and a 4x1
# quaternion respectively) return a 3x4 RT matrix
def convertToRT(trans, rot):                                                    
  '''Convert the translation vector and quaternion rotation that come from      
     tf by default to a 3x4 RT matrix'''                                        
  RT = np.zeros((3,4))
  RT[0:3, 0:3] = tf.transformations.quaternion_matrix(rot)[0:3, 0:3]
  RT[:,-1] = trans
  return RT

# Convert pythonic timestamp, trans (3x1) and rot (4x1) back into a 
# TransformStamped rosmsg
def convertToStampedTransform(ts, trans, rot):                                  
  '''Convert a timestamp, translation vector (3x1) and rotation quaternion      
     (4x1) into a ros TransformStamped message'''
  transform_msg = TransformStamped()
  transform_msg.header.stamp = ts
  transform_msg.transform.translation.x = trans[0]
  transform_msg.transform.translation.y = trans[1]
  transform_msg.transform.translation.z = trans[2]
  transform_msg.transform.rotation.x = rot[0]
  transform_msg.transform.rotation.y = rot[1]
  transform_msg.transform.rotation.z = rot[2]
  transform_msg.transform.rotation.w = rot[3]
  return transform_msg
  # NOTE: The version below, which relies on other geometry_msgs is a bit slower
  # due to the need to call multiple constructors
#  # Set translation
#  trans = Vector3(*trans)
#  # set rotation
#  rot = Quaternion(*rot)
#  # setup transform and tag with time stamp
#  transform = Transform(translation=trans, rotation=rot)
#  transform_msg = TransformStamped(transform=transform)
#  transform_msg.header.stamp = ts
#  return transform_msg

def convertQuaternionToTuple(quat):
  '''Convert a geometry_msg.msg.Quaternion into a 4x1 tuple'''
  return (quat.x, quat.y, quat.z, quat.w)

def convertTranslationToTuple(trans):
  '''Convert a geometry_msg.msg.Vector3 into a 3x1 tuple'''
  return (trans.x, trans.y, trans.z)
