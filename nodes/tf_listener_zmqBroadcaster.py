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

# Threshold for maximum distance between two adjacent pose estimates. If the
# computed distance between the poses is greater than this threshold, reject
# it
MAX_DIST = .20

if __name__ == '__main__':
    # Init rospy node
    rospy.init_node('tf_listener')
    listener = tf.TransformListener()
    emitter = ZmqEmitter("5557", 'poses')
    RTdata = np.empty(0, dtype=ttRType)
    ctr = 0
    rate = rospy.Rate(10.0)
#    # Open RT_time.txt for writing
#    fpath = os.path.expanduser('~')+'/Desktop/RT_%s.txt' %( rospy.get_time() )
#    fout = open( fpath, 'w' )

    outstr = '#timestamp\ttx\tty\ttz\tR00\tR01\tR02\tR10\tR11\tR12\tR20\tR21\tR22\n'
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
	# Handle the first pose estimate
	if len(RTdata) < 1:
	    RTdata = np.concatenate((RTdata, newPose))
	    continue
        last_RT = RTdata[-1]
	# Check against distance threshold in attempt to enforce a smooth
	# detector track
	inter_pose_dist = np.sqrt( (newPose['tx'] - last_RT['tx'])**2 +\
			           (newPose['ty'] - last_RT['ty'])**2 +\
				   (newPose['tz'] - last_RT['tz'])**2 )
	if inter_pose_dist > MAX_DIST: continue
	# HACK - Check for stopping condition
	if newPose['tx'] != last_RT['tx'] or newPose['ty'] != last_RT['ty'] or\
	   newPose['tz'] != last_RT['tz']:
            RTdata = np.concatenate((RTdata, newPose))
            # Emit the data, but only once a second
            if ctr % 10 == 0:
              emitter.send_zipped_pickle(RTdata)
            ctr += 1
        
            # Write to .ros/log std-log
            outstr += '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n'\
		      %(ts,trans[0],trans[1],trans[2],rot[0][0],rot[0][1],\
		      rot[0][2],rot[1][0],rot[1][1],rot[1][2],rot[2][0],\
		      rot[2][1],rot[2][2])

        rate.sleep()
#    print 'Writing to file...'
#    fout.write(outstr)
#    print 'Done.'
