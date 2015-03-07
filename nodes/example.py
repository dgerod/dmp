#!/usr/bin/env python

import roslib; roslib.load_manifest('dmp')
import rospy 

from dmp.srv import *
from dmp.msg import *
from dmpClient import makeLFDRequest, makeSetActiveRequest, makePlanRequest

import numpy as np

# --------------------------------------------------------------------------------------------------

if __name__ == '__main__':
  
    rospy.init_node('dmp_example')

    # Create a DMP from a 2-D trajectory ---
    
    K = 100                 
    D = 2.0 * np.sqrt(K)      
    dt = 1.0                
    
    dims = 2                
    num_bases = 4          
    traj = [[1.0,1.0],[2.0,2.0],[3.0,4.0],[6.0,8.0]]
    
    resp = makeLFDRequest(dims, traj, dt, K, D, num_bases)

    print resp
    
    # Set it as the active DMP ---
    
    makeSetActiveRequest(resp.dmp_list)

    # Now, generate a plan ---
    
    x_0 = [0.0,0.0]          # Plan starting at a different point than demo 
    x_dot_0 = [0.0,0.0]   
    t_0 = 0                
    goal = [8.0,7.0]         # Plan to a different goal than demo
    tau = 2 * resp.tau       # Desired plan should take twice as long as demo
    dt = 1.0
    
    goal_thresh = [0.2,0.2]
    seg_length = -1          # Plan until convergence to goal
    integrate_iter = 5       # dt is rather large, so this is > 1  
    
    plan = makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh, 
                           seg_length, tau, dt, integrate_iter)

    print plan
