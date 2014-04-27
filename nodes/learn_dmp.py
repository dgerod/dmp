#!/usr/bin/env python
import roslib; roslib.load_manifest('dmp')
import rospy 
import numpy as np
from dmp.srv import *
from dmp.msg import *

#Learn a DMP from demonstration data
def makeLFDRequest(dims, traj, dt, K_gain, 
                   D_gain, num_bases):
    demotraj = DMPTraj()
        
    for i in range(len(traj)):
        pt = DMPPoint();
        pt.positions = traj[i]
        demotraj.points.append(pt)
        demotraj.times.append(dt*i)
            
    k_gains = [K_gain]*dims
    d_gains = [D_gain]*dims
        
    print "Starting LfD..."
    rospy.wait_for_service('learn_dmp_from_demo')
    try:
        lfd = rospy.ServiceProxy('learn_dmp_from_demo', LearnDMPFromDemo)
        resp = lfd(demotraj, k_gains, d_gains, num_bases)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    print "LfD done"    
            
    return resp;

if __name__ == '__main__':
  
    rospy.init_node('dmp_tutorial_node')

    # Create a DMP from a 2-D trajectory
    dims = 2                
    dt = 1.0                
    K = 100                 
    D = 2.0 * np.sqrt(K)      
    num_bases = 4          
    traj = [[1.0,1.0],[2.0,2.0],[3.0,4.0],[6.0,8.0]]
    resp = makeLFDRequest(dims, traj, dt, K, D, num_bases)

    print resp
