#!/usr/bin/python
import os,sys, getopt

def main(argv):
  left = False
  right = False
    
  # get controller names
  ctrl_types =["_position_controller", "_velocity_controller", "_effort_controller"]
  joint_names = ["shoulder_pan",
                 "shoulder_lift",
                  "upper_arm_roll",
                  "elbow_flex",
                  "forearm_roll",
                  "wrist_flex",
                  "wrist_roll"]

 
  # load all parameters
  for ctrl_type in ctrl_types:
    os.system("rosparam load launch/pr2_joint" + ctrl_type +"s.yaml")
  print "rosparam loaded"

  controllers = []
  for ctrl_type in ctrl_types:
    for joint in joint_names:
      controllers.append("r_"+joint+ctrl_type)
      controllers.append("l_"+joint+ctrl_type)
  run = "rosrun pr2_controller_manager pr2_controller_manager "

  # load controllers
  for ctrl in controllers:
    os.system(run + " load " + ctrl)
  
  # stop default controllers
  os.system(run + " stop r_arm_controller")
  os.system(run + " stop l_arm_controller")
 
main(sys.argv[1:])
