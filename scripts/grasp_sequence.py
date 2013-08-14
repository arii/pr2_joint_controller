#!/usr/bin/env python
import roslib; roslib.load_manifest('my_controller_pkg')
import rospy
#import new_joint_controller as joint_controller # change this
from joint_controller import Cmd, ArmController, JOINTS, Joint
from my_controller_pkg.msg import MaxCurrentReached
import math,sys
rospy.init_node('grasp_sequence')


max_force = 10
velocity = -0.1



def sequential():

  arm = ArmController('l', 'position')
  arm_r = ArmController('r', 'position')

  pre_grasp_l = [math.pi/3, 0, math.pi/2, 0,0,0,0]
  pre_grasp_r = [0]*7; pre_grasp_r[0] = -math.pi/6
  arm.controlledMoveArm(pre_grasp_l)
  arm_r.controlledMoveArm(pre_grasp_r)
  raw_input("proceed to grasp sequence")
  r = rospy.Rate(10)
  flex_joints = (0,3,5)
  # change flex controllers to velocity control
  for i in flex_joints:
    joint_name = JOINTS[i]
    joint = arm.joints[joint_name]
    joint.setMaxEffort(max_force)
    joint.setController('velocity')
    joint.resetCollisions()
    joint.sendCommand(velocity/2)
    
    while not joint.collision_detected:
      r.sleep()
    
    print "%s- collision detected!" %joint_name
    joint.sendCommand(0)
  
  raw_input("all collsiions detected, about to start squeeze")
  arm.joints[JOINTS[0]].setController('position')
  arm.joints[JOINTS[3]].setController('effort')
  arm.joints[JOINTS[5]].setController('effort')
  arm.joints[JOINTS[3]].sendCommand(-10)
  arm.joints[JOINTS[5]].sendCommand(-10)
  """
  for i in flex_joints:
    joint = arm.joints[JOINTS[i]]
    joint.setController('effort')
    joint.sendCommand(-10)
  """
  raw_input("enter to quit")

  arm.joints[JOINTS[0]].setController('effort')
  for i in flex_joints:
    joint = arm.joints[JOINTS[i]]
    joint.sendCommand(0)


  #arm.moveJoints(cmds)
    
    

def open_loop():
  #rospy.init_node('grasp_sequence')
  
  # start in position control
  arm =ArmController('l', 'position') 
  arm_r = ArmController('r','position')

  # move to initial position 
  raw_input("Click enter to move to initial position")
 
  #pre_grasp_pose_l= [0.47253329544302414, -0.1384940200433169, 1.6428023092981965, -0.6065055700155089, -0.22774578310336668, -1.1253904163325368, 0.11072978819613777]
 
  pre_grasp_pose_l = [math.pi/4, 0, math.pi/2, 0, 0, 0, 0]
  arm.controlledMoveArm(pre_grasp_pose_l)
  pre_grasp_pose_r = [0]*7
  pre_grasp_pose_r[0] = -0.5
  arm_r.controlledMoveArm(pre_grasp_pose_r) 
  raw_input("intialize joint sequence.  press enter once joint controllers are started")
  print JOINTS 
  # generate commands for shoulder_pan, elbow_flex, and wrist_flex then set command to -1
  cmds = []
  for i in (0,3,5):
    cmd = Cmd(JOINTS[i],'velocity')
    arm.setJointController(cmd)
    cmd.val = -0.1
    cmds.append(cmd)
   
  arm.moveJoints(cmds)
  arm_r.setJointController(Cmd(JOINTS[0], 'velocity'))
  arm_r.moveJoint(Cmd(JOINTS[0],0.1))
  r = rospy.Rate(100)
  print "entering loop"
  quit=False
  while not quit:
    for joint in JOINTS:
      if arm.joints[joint].collision_detected:
      #if arm.collisionDetected():
        print "collision detected!"
        print joint
      #arm.setAllControllers('position')
      #arm_r.setAllControllers('position')
        quit=True
    r.sleep()

params = len(sys.argv) >1
if params and sys.argv[1] == '-sequential':
  sequential()   
elif params and sys.argv[1] == '-open_loop':
  open_loop()
else :
  print "USAGE: -sequential or -open_loop"
  sys.exit(2)

