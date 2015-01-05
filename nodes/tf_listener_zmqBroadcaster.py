#! /usr/bin/python
import numpy as np
import os
#import roslib
#roslib.load_manifest('rgbdslam')
import rospy
import math
import tf

# For zmq
import sys
from ZmqClass import ZmqEmitter

ttRType = np.dtype( {"names":['timestamp', 'tx', 'ty', 'tz', 'ROO', 'R01',\
                             'R02', 'R10', 'R11', 'R12', 'R20', 'R21', 'R22'],
                     "formats":[np.float64,np.float32,np.float32,np.float32,\
                                np.float32,np.float32,np.float32,np.float32,\
                                np.float32,np.float32,np.float32,np.float32,\
                                np.float32]} )

if __name__ == '__main__':
    # Init rospy node
    rospy.init_node('tf_listener')
    listener = tf.TransformListener()
    emitter = ZmqEmitter("5557", 'poses')
    RTdata = np.empty(0, dtype=ttRType)
    ctr = 0
    rate = rospy.Rate(10.0)
    # Open RT_time.txt for writing
    fpath = '/home/grim5/Desktop/RT_%s.txt' %( rospy.get_time() )
    fout = open( fpath, 'w' )

    outstr = '#timestamp	tx	ty	tz	R00	R01	R02	R10	R11	R12	R20	R21	R22\n'
    while not rospy.is_shutdown():
        try:
            ts = rospy.get_time()
            (trans,rot) = listener.lookupTransform('/camera_link', '/map',\
                                                   rospy.Time(0) )
        except (tf.LookupException, tf.ConnectivityException,\
                tf.ExtrapolationException):
            continue
	# Convert pose to RT
        rot = tf.transformations.quaternion_matrix( rot )
        R = rot[0:3, 0:3]
        # Format data properly for broadcasting
        newPose = np.array( [(ts,)+trans+tuple(R.flatten())],\
                            dtype=ttRType )
        RTdata = np.concatenate( (RTdata, newPose) )
        # Emit the data, but only once a second
        if ctr % 10 == 0:
          emitter.send_zipped_pickle(RTdata)
        ctr += 1
        
        # Write to .ros/log std-log
        outstr += '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' %(ts,trans[0],trans[1],trans[2],rot[0][0],rot[0][1],rot[0][2],rot[1][0],rot[1][1],rot[1][2],rot[2][0],rot[2][1],rot[2][2])

        rate.sleep()
    print 'Writing to file...'
    fout.write(outstr)
    print 'Done.'
