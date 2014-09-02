#!/usr/bin/env python
# ==================================================================================================
# dgerod@xyz-lab.org.es - 2014
# --------------------------------------------------------------------------------------------------
# Plan a trajectory using DMP data.
# ==================================================================================================

import sys

import roslib; roslib.load_manifest("dmp")
import rospy 
import numpy as np

from dmp.srv import *
from dmp.msg import *

import rosbag, yaml

# --------------------------------------------------------------------------------------------------

# Set a DMP as active for planning
def makeSetActiveRequest(dmp_list):
    try:
        sad = rospy.ServiceProxy("set_active_dmp", SetActiveDMP)
        sad(dmp_list)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# Generate a plan from a DMP
def makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh, 
                    seg_length, tau, dt, integrate_iter):
    print "Starting DMP planning..."
    rospy.wait_for_service('get_dmp_plan')
    try:
        gdp = rospy.ServiceProxy('get_dmp_plan', GetDMPPlan)
        resp = gdp(x_0, x_dot_0, t_0, goal, goal_thresh, 
                   seg_length, tau, dt, integrate_iter)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    print "DMP planning done"   
            
    return resp;

# --------------------------------------------------------------------------------------------------

if __name__ == '__main__':
  
    rospy.init_node('plan_trj_using_dmp')

    # Read bag where DMP data is stored (e.g. "dmp-data.bag")
    # --------------------------------------------------------
    
    fileBag = sys.argv[1]
     
    bag = rosbag.Bag( fileBag, "r" );
    for topic, msg, t in bag.read_messages(topics=['LearnDMPFromDemoResponse']):
      print msg
    bag.close();    
    
    resp = msg;
    
    # Set it as the active DMP
    # --------------------------------------------------------

    num_bases = len(resp.dmp_list[0].weights)
    dims = len(resp.dmp_list)    
    print "num bases: %d, dims: %d" % (num_bases, dims)
  
    makeSetActiveRequest(resp.dmp_list)

    # Generate a plan based on DMP
    # --------------------------------------------------------

    # Default parameters (2D trajectory)
    x_0 = [0.0,0.0]          # Plan starting at a different point than demo 
    x_dot_0 = [0.0,0.0]   
    t_0 = 0                
    goal = [8.0,7.0]         # Plan to a different goal than demo
    
    goal_thresh = [0.2,0.2]
    seg_length = -1          # Plan until convergence to goal
    tau = 2 * resp.tau       # Desired plan should take twice as long as demo
    dt = 1.0
    integrate_iter = 5       # dt is rather large, so this is > 1
    
    # Read parameters (e.g. start, goal) from file (e.g. "plan-cfg.yaml")
    try:

      cfgFile = sys.argv[2]

      cfg = open(cfgFile)
      dataMap = yaml.safe_load(cfg)
      cfg.close()
      
      print dataMap
        
      x_0 = dataMap['start']['positions']
      x_dot_0 = dataMap['start']['velocities']
      t_0 = dataMap['start']['time']
      goal = dataMap['goal']['positions']      

    except:
      raise 
    
    # Now, generate a plan using loaded parameters    
    resp = makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh, seg_length, tau, dt, integrate_iter)
    print resp
    
    num_bases = len(resp.plan.points)
    dims = len(resp.plan.points[0].positions)    
    print "num bases: %d, dims: %d" % (num_bases, dims)
      
    # Export plan (new trajectory) to another bag (e.g. "plan-trj.bag")
    # --------------------------------------------------------
    
    fileBag = sys.argv[3]
    
    bag = rosbag.Bag(fileBag, "w");
    bag.write("DMPTraj", resp.plan)
    bag.close()
    
# ==================================================================================================
