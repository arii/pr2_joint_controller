#!/usr/bin/python
import os,sys, getopt

def main(argv):
  
  if (len(argv) == 0):
    print "argument should be position, effort, or velocity"
    sys.exit(2)
  # get controller names
  if(argv[0] == 'position'):
    ctrl_types = ["_position_controller"]
    not_ctrl_types =["_effort_controller", "_velocity_controller"]
  elif(argv[0] == 'velocity'):
    ctrl_types = ["_velocity_controller"]
    not_ctrl_types =["_effort_controller", "_position_controller"]
  elif(argv[0] == 'effort'): 
    not_ctrl_types =["_position_controller", "_velocity_controller"]
    ctrl_types =[ "_effort_controller"]
  else:
    print "2nd argument should be position, effort, or velocity"
    sys.exit(2)
  
  joint_names = ["shoulder_pan",
                 "shoulder_lift",
                  "upper_arm_roll",
                  "elbow_flex",
                  "forearm_roll",
                  "wrist_flex",
                  "wrist_roll"]
  controllers = []
  for ctrl_type in ctrl_types:
    for joint in joint_names:
      controllers.append("r_"+joint+ctrl_type)
      controllers.append("l_"+joint+ctrl_type)
  # stop not_ctrl_types
  run = "rosrun pr2_controller_manager pr2_controller_manager "
  for ctrl_type in not_ctrl_types:
    for joint in joint_names:
      os.system(run + "stop r_" + joint + ctrl_type + " > /dev/null")
      os.system(run + "stop l_" + joint + ctrl_type + " > /dev/null")
  # start controllers
  for ctrl in controllers:
    os.system(run + " start " + ctrl + " > /dev/null")
  print "started controllers"

main(sys.argv[1:])
