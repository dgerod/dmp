#!/usr/bin/env python

# ==================================================================================================
# dgerod.xyz-lab.org.es - 2014
# --------------------------------------------------------------------------------------------------
# Plan a trajectory using DMP data.
# ==================================================================================================

import roslib; 
roslib.load_manifest('dmp')
import rospy 
import numpy as np
from dmp.srv import *
from dmp.msg import *

import rosbag

# --------------------------------------------------------------------------------------------------

#Set a DMP as active for planning
def makeSetActiveRequest(dmp_list):
    try:
        sad = rospy.ServiceProxy('set_active_dmp', SetActiveDMP)
        sad(dmp_list)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

#Generate a plan from a DMP
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

    # Read bag where DMP data is stored
    # --------------------------------------------------------
    
    bag = rosbag.Bag("dmp_data.bag");
    for topic, msg, t in bag.read_messages(topics=['LearnDMPFromDemoResponse']):
      print msg
    bag.close();    
  
    resp = msg;
        
    # Set it as the active DMP
    # --------------------------------------------------------
    
    makeSetActiveRequest(resp.dmp_list)

    # Generate a plan based on DMP
    # --------------------------------------------------------
    
    # Prepare parameters
    x_0 = [0.0,0.0]          # Plan starting at a different point than demo 
    x_dot_0 = [0.0,0.0]   
    t_0 = 0                
    goal = [8.0,7.0]         # Plan to a different goal than demo
    goal_thresh = [0.2,0.2]
    seg_length = -1          # Plan until convergence to goal
    tau = 2 * resp.tau       # Desired plan should take twice as long as demo
    dt = 1.0
    integrate_iter = 5       # dt is rather large, so this is > 1  

    # Now, generate a plan    
    plan = makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh, seg_length, tau, dt, integrate_iter)

    print plan

# ==================================================================================================
