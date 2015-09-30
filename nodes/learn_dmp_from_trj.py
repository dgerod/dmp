#!/usr/bin/env python
# ========================================================================================
# dgerod@xyz-lab.org.es - 201x
# ----------------------------------------------------------------------------------------
# Learn DMP data from a trajectory, it could be PTP or Cartesian.
#
# INPUTS:
#   - traj_bag_file   : argv[1] - Bag where is the trajectory.
#   - topic_name      : argv[2] - Topic used in the bag ("/joint_states" or "/pose").
#   - dmp_params_file : argv[3] - Parameters to configure the mDMP.
#   - dmp_bag_file    : argv[4] - Bag where learned mDMP is exported.   
# ========================================================================================

import sys, os

import roslib; roslib.load_manifest("dmp")
import rospy
import rosbag

from dmpClient import makeLFDRequest
from fileHelper import readPtpTrajectory, readCartTrajectory, readHandTrajectory
from fileHelper import readDmpParameters, writeDmpData

import numpy as np

# ----------------------------------------------------------------------------------------

if __name__ == "__main__":

    # Initialize node
    # -------------------------------------------------------
    
    print sys.argv

#   THIS DOES NOT WORK WHEN THIS NOTE IS EXECUTED FROM PYTHON USING ROSLAUNCH.
#       
#   if len(sys.argv) != 5:        
#       sys.exit("Usage: %s traj-bag-file topic-name dmp-params-file dmp-bag-file" 
#                % os.path.basename(sys.argv[0]))

    rospy.init_node("learn_dmp_from_trj")

    # Read trajectory from a topic (e.g. "/joint_states") in bag file 
    # (e.g. "robot_joints.bag")
    # -------------------------------------------------------
    # The name of the topic is used to decide which is the type of trajectory.  
    
    traj_bag_file = sys.argv[1]
    topic_name = sys.argv[2]
    
    rospy.loginfo("Trajectory bag: %s", traj_bag_file)
    rospy.loginfo("Topic name: %s", topic_name)
    
    if topic_name == "/joint_states":
      # Read a trajectory as a list of "JointStates" messages.  
      traj, dims = readPtpTrajectory(traj_bag_file)
    elif topic_name == "/pose":
      # Read a trajectory as a list of "PoseStamped" messages.
      traj, dims = readCartTrajectory(traj_bag_file)
    else:
      # Read a trajectory as a list of "PoseStamped" messages.
      traj, dims = readHandTrajectory(traj_bag_file)
    
    if dims == 0:
      raise IOError("Problem loading the trajectory")
    
    print traj
    print "Trajectory - num samples %d, dims: %d" % (len(traj), dims)
    
    # Create a DMP from the loaded trajectory
    # --------------------------------------------------------
    
    dmp_params_file = sys.argv[3] 
    rospy.loginfo("DMP parameters: %s", dmp_params_file)
    
    num_samples, dt, K, D, num_bases = readDmpParameters(dmp_params_file)
    print "mDMP parameters - ns: %d, bfs: %d, K: %f, D: %f, dt: %f" % (num_samples, num_bases, K, D, dt)
        
    resp = makeLFDRequest(dims, traj, dt, num_samples, K, D, num_bases)    
    print resp
  
    # Export DMP to another bag (e.g. "dmp-data.bag")
    # --------------------------------------------------------
    
    dmp_bag_file = sys.argv[4]   
    rospy.loginfo("DMP data bag: %s", dmp_bag_file)
    
    writeDmpData(dmp_bag_file, resp)

# ========================================================================================
