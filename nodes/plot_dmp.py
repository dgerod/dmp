# ========================================================================================
#
# ========================================================================================

import sys, os
import rosbag

import numpy as np
import matplotlib.pylab as plt
from argparse import ArgumentError

# ----------------------------------------------------------------------------------------

def plot_cart_trajectory(bag_name):
    
    bag = rosbag.Bag(bag_name, "r");    
    poses = []
    for topic, msg, t in bag.read_messages("/pose"):
        position = np.array( [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z] )    
        orientation = [0, 0, 0]
        poses.append( np.concatenate([position, orientation]) )    
    bag.close()   
    print poses

    trajectory = np.array(poses).squeeze()
    print trajectory
        
    plt.figure()
    title = "DMP - Trajectory (Pose) - %s" % (bag_name)    
    plt.plot(trajectory)
            
    plt.savefig( bag_name + "_cart-tj.png" )
    
    plt.figure()
    plt.plot(trajectory)
    
def plot_ptp_trajectory(bag_name):
    
    bag = rosbag.Bag(bag_name, "r");    
    jnts = []
    for topic, msg, t in bag.read_messages("/joint_states"):
        jnts.append( np.array(msg.position) )        
    bag.close()   
    print jnts
    
    trajectory = np.array(jnts).squeeze()
    print trajectory
        
    plt.figure()
    title = "DMP - Trajectory (Joints) - %s" % (bag_name)    
    plt.plot(trajectory)
            
    plt.savefig( bag_name + "_ptp-tj.png" )
      
def plot_dmp (bag_name):
    
    bag = rosbag.Bag(bag_name, "r");
    for topic, msg, t in bag.read_messages("LearnDMPFromDemoResponse"):
        dmps = msg.dmp_list
    bag.close()    
    
    weights = []
    f_targets = []               
    for sdx in range(len(dmps) ):
        weights.append(np.array( dmps[sdx].weights ))    
        f_targets.append(np.array( dmps[sdx].f_targets ))
    print weights        
    print f_targets
           
    weights = np.array(weights)
    f_targets = np.array(f_targets)    
          
    plt.figure()
    title = "DMP - Weights - %s" % (bag_name)
    plt.plot(weights.T)
                    
    plt.savefig( bag_name + "_dmp-weigths.png" )

    plt.figure()
    title = "DMP - f(s) - %s" % (bag_name)
    plt.plot(f_targets.T)
            
    plt.savefig( bag_name + "_dmp-ft.png" )
    
def plot_2d_plan(bag_name):
    
    bag = rosbag.Bag(bag_name, "r");
    for topic, msg, t in bag.read_messages("DMPTraj"):
        points = msg.points
    bag.close()    
    
    pose = []
    for pdx in range( len(points) ):
        pose.append( np.array(points[pdx].positions[0:2]) )
    print pose
    
    trajectory = np.array(pose).squeeze()
    print trajectory
        
    plt.figure()
    title = "DMP - Generated plan (Pose) - %s" % (bag_name)    
    plt.plot(trajectory)
                    
    plt.savefig( bag_name + "_cart-plan_time.png" )

    plt.figure()
    title = "DMP - Generated plan (Pose)- %s" % (bag_name)    
    plt.plot(trajectory[:,0], trajectory[:,1])
                    
    plt.savefig( bag_name + "_cart-plan_xy.png" )

def plot_ptp_plan(bag_name):
    
    bag = rosbag.Bag(bag_name, "r");
    for topic, msg, t in bag.read_messages("DMPTraj"):
        points = msg.points
    bag.close()    
    
    jnts = []
    for pdx in range( len(points) ):
        jnts.append( np.array(points[pdx].positions) )
    print jnts
    
    trajectory = np.array(jnts).squeeze()
    print trajectory
        
    plt.figure()
    title = "DMP - Generated plan (Joints) - %s" % (bag_name)    
    plt.plot(trajectory)
                    
    plt.savefig( bag_name + "_ptp-plan_time.png" )

# ----------------------------------------------------------------------------------------
    
if __name__ == '__main__':
               
    if len(sys.argv) != 3:        
        sys.exit("Usage: %s [ptp|cartesian] prefix-file-name" 
                 % os.path.basename(sys.argv[0]))
  
    traj_type = sys.argv[1]
    prefix_name = sys.argv[2]
    
    if traj_type == "ptp":
        plot_ptp_trajectory(prefix_name + "_trj-joints.bag")
        plot_dmp(prefix_name + "_dmp.bag")
        plot_ptp_plan(prefix_name + "_plan-joints.bag")
        
    elif traj_type == "cartesian":
        plot_cart_trajectory(prefix_name + "_trj-poses.bag")
        plot_dmp(prefix_name + "_dmp.bag")
        plot_2d_plan(prefix_name + "_plan-poses.bag")
    
    else:
        raise ArgumentError("Trajectory type must be 'ptp' or 'cartesian'")

# ========================================================================================