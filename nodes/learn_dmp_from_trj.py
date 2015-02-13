#!/usr/bin/env python
# ==================================================================================================
# dgerod@xyz-lab.org.es - 2014
# --------------------------------------------------------------------------------------------------
# Learn DMP data from a trajectory.
# ==================================================================================================

import sys

import roslib; roslib.load_manifest("dmp")
import rospy 
import rosbag

from dmp.srv import *
from dmp.msg import *
from dmpClient import makeLFDRequest
from fileHelper import readPtpTrajectory, readCartTrajectory, writeDmp

import numpy as np

# --------------------------------------------------------------------------------------------------

if __name__ == "__main__":
  
    rospy.init_node("learn_dmp_from_trj")

    # Read trajectory from a topic ( e.g. "/joint_states") in bag file 
    # (e.g. "robot_joints.bag")
    # -------------------------------------------------------
    # The name of the topic is used to decide which is the type of trajectory.  
    
    traj_bag_file = sys.argv[1]
    topic_name = sys.argv[2]

    if topic_name == "/joint_states":
      # Read a trajectory from a bag of "JointStates" messages.  
      traj, dims = readPtpTrajectory(traj_bag_file, topic_name)
    else:
      # Read a trajectory from a bag of "PoseStamped" messages.
      traj, dims = readCartTrajectory(traj_bag_file, topic_name)
    
    if dims == 0:
      raise IOError("Problem loading the trajectory")
    
    print traj
    print "num samples %d, dims: %d" % (len(traj), dims)
    
    # Create a DMP from the loaded trajectory
    # --------------------------------------------------------
    
    # TODO - dgerod: load parameters from YAML file. 
    num_bases, K, D, dt = readDmpParameters("")
        
    num_bases = 80
    print "num bases: %d, K: %f, D: %f, dt: %f" % (num_bases, K, D, dt)
        
    resp = makeLFDRequest(dims, traj, dt, K, D, num_bases)    
    print resp
  
    # Export DMP to another bag (e.g. "dmp-data.bag")
    # --------------------------------------------------------
    
    dmp_bag_file = sys.argv[3]   
    writeDmp(dmp_bag_file, resp)

# ==================================================================================================
