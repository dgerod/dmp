#!/usr/bin/env python
# ========================================================================================
# dgerod@xyz-lab.org.es - 2014
# ----------------------------------------------------------------------------------------
# Plan a trajectory using mDMP data.
#
# INPUTS:
#   - dmp_bag_file  : argv[1] - Bag where is the mDMP previously learned.
#   - plan_cfg_file : argv[2] - Parameters to configure the plan.
#   - plan_bag_file : argv[3] - Bag where planned trajectory is exported.       
# ========================================================================================

import sys, os

import roslib; roslib.load_manifest("dmp")
import rospy 
import rosbag, yaml

from fileHelper import readPlanConfiguration, readDmp, writePlannedTrajectory
from dmpClient import makeSetActiveRequest, makePlanRequest

import numpy as np

# ----------------------------------------------------------------------------------------

if __name__ == "__main__":

    # Initialize node
    # -------------------------------------------------------
        
    if len(sys.argv) != 4:        
        sys.exit("Usage: %s dmp-bag-file plan-cfg-file plan-bag-file" 
                 % os.path.basename(sys.argv[0]))

    rospy.init_node("learn_dmp_from_trj")
    
    # Read bag where DMP data is stored (e.g. "dmp-data.bag")
    # --------------------------------------------------------
    
    dmp_bag_file = sys.argv[1]
    resp = readDmp(dmp_bag_file)
    print resp 
    
    # Set it as the active DMP
    # --------------------------------------------------------

    num_bases = len(resp.dmp_list[0].weights)
    dims = len(resp.dmp_list)  
    print "dims: %d, num bases: %d" % (dims, num_bases)
  
    makeSetActiveRequest(resp.dmp_list)

    # Generate a plan based on DMP
    # --------------------------------------------------------

    plan_cfg_file = sys.argv[2]    
    print "Plan configuration: ", plan_cfg_file 
    
    # Read plan parameters (e.g. start, goal) from file (e.g. "plan-cfg.yaml")
    x_0, x_dot_0, goal, t_0, tau = readPlanConfiguration(plan_cfg_file, resp.tau)

    goal_thresh = [0.2,0.2]
    seg_length = -1          # Plan until convergence to goal
    dt = 1.0
    integrate_iter = 5       # dt is rather large, so this is > 1
    
    # Now, generate a plan using loaded parameters    
    resp = makePlanRequest(x_0, x_dot_0, t_0, goal, 
                           goal_thresh, seg_length, tau, dt, integrate_iter)
    print resp
    
    num_bases = len(resp.plan.points)
    dims = len(resp.plan.points[0].positions)     
    print "num bases: %d, dims: %d" % (num_bases, dims)
      
    # Export plan (new trajectory) to another bag (e.g. "plan-trj.bag")
    # --------------------------------------------------------
    
    plan_bag_file = sys.argv[3]    
    writePlannedTrajectory(plan_bag_file, resp.plan)

# ========================================================================================
