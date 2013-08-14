#!/usr/bin/env python
# controller package for my custon controllers
import roslib; roslib.load_manifest('my_controller_pkg')
import rospy
from my_controller_pkg.srv import \
    Command, CtrlTypeSrv, IsMaxReached, MaxEffort
from my_controller_pkg.msg import ControllerState
import sys, thread, Queue, math

JOINTS = ["shoulder_pan",
          "shoulder_lift",
          "upper_arm_roll",
          "elbow_flex",
          "forearm_roll",
          "wrist_flex",
          "wrist_roll"]

# declare hystersis boundaries
FAST_MODE = False
if(FAST_MODE):
  REST_MOVING = 50
  MOVING_REST = 70
else:
  REST_MOVING = 15
  MOVING_REST = 20

def errorMsg(msg):
  print "ERROR:" + msg
  sys.exit(2)

def isValidCtrl(ctrl):
  return (ctrl == 'position' or ctrl=='velocity' or ctrl=='effort')

def isValidArm(which_arm):
  return  (which_arm[0] == 'l' or which_arm[0] == 'r')

class Joint:
  def __init__(self, which_arm, joint_name, ctrl_type):
    global JOINTS, UPPER_HYSTERSIS, LOWER_HYSTERSIS
    
    # init ctrl_type, joint_type, and controller

    if isValidCtrl(ctrl_type):
      self.ctrl_type = ctrl_type
    else:
      errorMsg( "%s is the wrong controller type" % ctrl_type )
    
    if joint_name in JOINTS:
      self.joint_type = joint_name
    else:
      errorMsg("%s is not a valid joint" % joint_name)

    if isValidArm(which_arm):
      self.controller = which_arm[0] + "_"\
          + joint_name + "_controller"
    else:
      errorMsg( "%s is not a valid arm" % which_arm)

    # if we made it this far, we're ready to make service proxies
    self.command = rospy.ServiceProxy(\
        self.controller+"/command", Command)
    self.ctrl_type_srv = rospy.ServiceProxy(\
        self.controller+"/ctrl_type",CtrlTypeSrv)
    self.is_max_reached_srv = rospy.ServiceProxy(\
        self.controller+"/is_max_reached",IsMaxReached)
    self.max_effort_srv = rospy.ServiceProxy(\
        self.controller+"/max_effort", MaxEffort)
    
    self.resting_state = True  # using upper,lower bounds to change 
    self.subscriber = ""
    self.last_msg = "" 
    thread.start_new_thread(self.getControllerState,())
    # make sure we are in the correct control
    self.setController(ctrl_type)
    self.collision_detected = False

  def resetCollisions(self):
    self.collision_detected = False
  
  def controllerStateCallback(self,data):
    self.last_msg = data
    if self.resting_state:
      if math.fabs(data.filtered_effort) < REST_MOVING:
        self.resting_state = False
        self.collision_detected = False
    else:
      if math.fabs(data.filtered_effort) > MOVING_REST:
        self.resting_state = True
        self.collision_detected = True

  def getControllerState(self):
    self.subscriber= rospy.Subscriber(\
        self.controller+"/controller_state",\
        ControllerState, self.controllerStateCallback)

  def setMaxEffort(self, max_effort):
    self.max_effort_srv(max_effort)
  
  def setController(self, ctrl_type):
    if isValidCtrl(ctrl_type):
      self.ctrl_type_srv(ctrl_type)
      self.ctrl_type = ctrl_type
    else:
      print "%s not a valid ctrl type" % ctrl_type
      print "controller is still %s" % self.ctrl_type
  
  def sendCommand(self, cmd):
    self.command(cmd)
  
  def controlledMove(self, val):
    self.setController('effort')
    r = rospy.Rate(10)
    error = self.last_msg.position - val
    while math.fabs(error) > 0.05:
      #print self.joint_type
      self.sendCommand(-10*error)
      error = self.last_msg.position - val
      r.sleep()
    self.setController('position')
    self.sendCommand(val)

class Cmd:
  def __init__(self,joint_name, val):
    self.name = joint_name
    self.val = val

# to send bulk commands to all joints
class ArmController:
  global JOINTS
  
  def __init__(self, which_arm, ctrl_type):
    self.joints = {}
    if isValidCtrl(ctrl_type) and isValidArm(which_arm):
      for joint_name in JOINTS:
        self.joints[joint_name] = Joint(\
            which_arm, joint_name, ctrl_type)
    else:
      errorMsg("incorrect arm or controller type")
  
  def setAllControllers(self, ctrl_type):
    if isValidCtrl(ctrl_type):
      for joint in JOINTS:
        self.joints[joint].setController(ctrl_type)
    else:
      print "invalid controller type %s. \
      Did not change controllers" % ctrl_type
  
  # set controller for a specific joint
  def setJointController(self, cmd):
    if cmd.name in JOINTS and isValidCtrl(cmd.val):
      self.joints[cmd.name].setController(cmd.val)
    else :
      print "invalid joint name or wrong controller type"
  
  def controlledMoveArm(self, values):
    for i in range(len(JOINTS)):
      if (i >1):
        self.joints[JOINTS[i]].sendCommand(values[i])
      else:
        param = values[i],
        thread.start_new_thread(self.joints[JOINTS[i]].controlledMove, param)

  # values is an array
  def moveArm(self,values):
    for i in range(len(JOINTS)):
      self.joints[JOINTS[i]].sendCommand(values[i])
  def moveJoint(self, cmd):
    #command is a MoveJointCommand
    if cmd.name in JOINTS:
      self.joints[cmd.name].sendCommand(cmd.val)
  
  def moveJoints(self, commands):
    #commands is an array of MoveJointCommand
    for cmd in commands:
      self.moveJoint(cmd)

  def collisionDetected(self):
    collided = False
    for name in JOINTS:
      if self.joints[name].isMaxReached():
        collided = True
    return collided

if __name__=="__main__":
  rospy.init_node("joint_ctrl_test")
  test = Joint('l','shoulder_pan','effort')
  
  """raw_input("test controlled move")
  test.controlledMove(0)
  raw_input("test next controlled move")
  test.controlledMove(1)
  raw_input("compare with uncontrolled")
  test.sendCommand(0)
  raw_input("compare with uncontrolled")
  test.sendCommand(1)

  raw_input("Single joint effort command")
  test.sendCommand(0.1)
  raw_input("click enter to test position control")
  test.setController("position")
  test.sendCommand(0)"""
  
  raw_input("testing arm controller") 
  arm = ArmController('l','position')
  print "the following tests show the difference between control move and non controlled move"
  raw_input("test controll move arm")
  arm.controlledMoveArm([0]*7)
  raw_input("test next controll move arm")
  arm.controlledMoveArm([1]*7)
  raw_input("compare with uncontrolled")
  arm.moveArm([0]*7) 
  raw_input("compare with uncontrolled")
  arm.moveArm([1]*7)
  
  raw_input("test position control")
  arm.setAllControllers('position')
  arm.moveArm([0]*7)
  raw_input("test individual joint control")
  arm.setJointController(Cmd('wrist_roll', 'velocity'))
  cmd1 = Cmd('wrist_roll',1)
  arm.moveJoint(cmd1)
  raw_input("testing move array of commands")
  cmds = [Cmd('wrist_flex',2),Cmd('shoulder_pan',.5), Cmd('elbow_flex',2)]
  arm.moveJoints(cmds)
  print "exiting" 
