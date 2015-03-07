# ========================================================================================
# dgerod@xyz-lab.org.es - 2014
# ----------------------------------------------------------------------------------------
# Load information from different types of files.
# ========================================================================================

import numpy as np
import rospy 
import rosbag, yaml

def readPtpTrajectory(bag_name):
    """ Open a bag file and loads JointStage messages from the /joint_states topic. """
  
    trajectory = []    

    bag = rosbag.Bag(bag_name, "r")
    for topic, msg, t in bag.read_messages("/joint_states"):
      trajectory.append(msg.position)
    bag.close()
        
    if len(trajectory) > 0:
      dims = len(trajectory[0])
    else:
      dims = 0
      
    return trajectory, dims
    
def readCartTrajectory(bag_name):
    """ Open a bag file and loads only the X,Y coordinates of PoseStamped message 
    from the /pose topic. """
    
    trajectory = []    

    bag = rosbag.Bag(bag_name, "r")
    for topic, msg, t in bag.read_messages("/pose"):
      trajectory.append( [msg.pose.position.x, msg.pose.position.y] )
    bag.close()
    
    if len(trajectory) > 0:
      dims = len(trajectory[0])
    else:
      dims = 0
            
    return trajectory, dims

def readDmpParameters(file_name):
  
    bfs = 80 
    K = 100                 
    D = 2.0 * np.sqrt(K)
    dt = 1.0
  
    # Read parameters from a file (e.g. "dmp-cfg.yaml")
    try:
        yaml_file = open(file_name)
        config_data = yaml.safe_load(yaml_file)
        yaml_file.close()
          
        bfs = config_data["bfs"]  
        K = config_data["K"]        
        D = 2.0 * np.sqrt(K)
        dt = config_data["dt"]  
                
    except:
        raise IOError("Problem loading mDMP parameters.") 
         
    return bfs, K, D, dt
    
def readPlanConfiguration(file_name, tau):
    """ Read the configuration of a plan from a defined YAML file. """
    
    # Default parameters (2D trajectory)
    x_0 = [0.0, 0.0]          # Plan starting at a different point than demo 
    x_dot_0 = [0.0, 0.0]   
    goal = [8.0, 7.0]         # Plan to a different goal than demo
    t_0 = 0                
    
    # Read parameters (e.g. start, goal) from a file (e.g. "plan-cfg.yaml")
    try:
        yaml_file = open(file_name)
        config_data = yaml.safe_load(yaml_file)
        yaml_file.close()
          
        x_0 = config_data["start"]["positions"]
        x_dot_0 = config_data["start"]["velocities"]
        t_0 = config_data["start"]["time"]
        goal = config_data["goal"]["positions"]   

    except:
        raise IOError("Problem loading plan configuration.") 
    
    # Desired plan should take twice as long as demo
    tau = 2 * tau      
         
    return x_0, x_dot_0, goal, t_0, tau

def readDmp(bag_name):
    bag = rosbag.Bag(bag_name, "r");
    for topic, msg, t in bag.read_messages("LearnDMPFromDemoResponse"):
        resp = msg
    bag.close()
    return resp 

def writeDmp(bag_name, resp):
    bag = rosbag.Bag(bag_name, "w");
    bag.write("LearnDMPFromDemoResponse", resp)
    bag.close()
  
def writePlannedTrajectory(bag_name, plan):      
    bag = rosbag.Bag(bag_name, "w");
    bag.write("DMPTraj", plan)
    bag.close()

# ========================================================================================
