# ========================================================================================
# dgerod@xyz-lab.org.es - 2014
# ----------------------------------------------------------------------------------------
# Learn and plan a mDMP calling the server that is a ROS node.
# ========================================================================================

import numpy as np
import rospy 
from dmp.srv import *
from dmp.msg import *

# ----------------------------------------------------------------------------------------

def selectFunctionApproxRequest():
    """ Set an already obtained mDMP as active for planning."""
    
    try:
        sad = rospy.ServiceProxy("select_function_approx", SetActiveDMP)
        sad(dmp_list)
        
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def makeLFDRequest(dims, traj, dt, K_gain, D_gain, num_bases):
    """ Learn a mDMP from demonstration data. """
    
    demo_traj = DMPTraj()        
    for i in range(len(traj)):
        pt = DMPPoint();
        pt.positions = traj[i]
        demo_traj.points.append(pt)
        demo_traj.times.append(dt*i)
            
    k_gains = [K_gain]*dims
    d_gains = [D_gain]*dims
        
    print "Starting learning..."
    rospy.wait_for_service("learn_dmp_from_demo")
    
    try:
        lfd = rospy.ServiceProxy("learn_dmp_from_demo", LearnDMPFromDemo)
        resp = lfd(demo_traj, k_gains, d_gains, num_bases)
        
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e
        
    print "Learning done."
    return resp;

def makeSetActiveRequest(dmp_list):
    """ Set an already obtained mDMP as active for planning."""
    
    try:
        sad = rospy.ServiceProxy("set_active_dmp", SetActiveDMP)
        sad(dmp_list)
        
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh, 
                    seg_length, tau, dt, integrate_iter):
    """ Generate a plan from a mDMP. """

    print "Starting planning..."
    rospy.wait_for_service('get_dmp_plan')
    
    try:
        gdp = rospy.ServiceProxy("get_dmp_plan", GetDMPPlan)
        resp = gdp(x_0, x_dot_0, t_0, goal, 
                   goal_thresh, seg_length, tau, dt, integrate_iter)
        
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e
    
    print "Planning done."
    return resp;

# ========================================================================================
