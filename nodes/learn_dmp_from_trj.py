#!/usr/bin/env python
# ==================================================================================================
# dgerod@xyz-lab.org.es - 2014
# --------------------------------------------------------------------------------------------------
# Learn DMP data from a trajectory.
# ==================================================================================================

import sys

import roslib; roslib.load_manifest("dmp")
import rospy 
import numpy as np

from dmp.srv import *
from dmp.msg import *
from dmp_client import makeLFDRequest

import rosbag

# --------------------------------------------------------------------------------------------------

if __name__ == "__main__":
  
    rospy.init_node("learn_dmp_from_trj")

    # Read trajectory from a topic ( e.g. "/joint_states") in bag/file 
    # (e.g. "joint-states.bag")
    # --------------------------------------------------------

    fileBag = sys.argv[1]
    topicName = sys.argv[2]
        
    bag = rosbag.Bag(fileBag, "r")
    
    traj = []    
    for topic, msg, t in bag.read_messages(topicName):
      traj.append(msg.position)
    
    dt = 1.0
    num_bases = len(traj)
    dims = len(msg.position)
    
    print traj
    print num_bases, dims
    print "num bases: %d, dims: %d" % (num_bases, dims)
    
    # Create a DMP from the loaded trajectory
    # --------------------------------------------------------
    
    K = 100                 
    D = 2.0 * np.sqrt(K)      
    
    resp = makeLFDRequest(dims, traj, dt, K, D, num_bases)    
    print resp
  
    # Export DMP to another bag (e.g. "dmp-data.bag")
    # --------------------------------------------------------
    
    fileBag = sys.argv[3]
    
    bag = rosbag.Bag(fileBag, "w");
    bag.write("LearnDMPFromDemoResponse", resp)
    bag.close()
    
# ==================================================================================================
