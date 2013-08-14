#!/usr/bin/env python
import roslib; roslib.load_manifest('my_controller_pkg')
import rospy
from joint_controller import FAST_MODE, Cmd, Joint, ArmController, JOINTS
from my_controller_pkg.msg import MaxCurrentReached
import thread, sys

# Joint to test:
rospy.init_node('joint_collision_detector')
joint_idx = 5
joint = Joint('l', JOINTS[joint_idx],'position')


def check_collision():
  if(FAST_MODE):
    max_eff = 10
    desired_vel = -1
  else:
    max_eff = 10
    desired_vel = -0.1
  raw_input("move to initial pose")
  joint.sendCommand(1)

  raw_input("Start close and detect collision mode")
  joint.setMaxEffort(max_eff)
  joint.setController("velocity")
  r = rospy.Rate(1)
  joint.sendCommand(desired_vel) 
  #thread.start_new_thread(GetUserInput, ())
  
  print "resting phase one"
  while joint.resting_state:
    r.sleep()
  print "robot is moving"
  while not joint.resting_state:
    r.sleep()
  print "collision detected!"
  joint.setController("position")

  #joint.sendCommand(0)
  print "staying in position control"
  

quit = False
def GetUserInput():
  global quit
  raw_input("Input anything to quit")
  quit = True

def capture_data():
  r = rospy.Rate(1000)
  global quit, joint

  joint.sendCommand(1)
  trial_name = raw_input("Trial name?")
  try:
    max_eff = float(raw_input("max effort?"))
  except:
    print "no input, max_eff = 25"
    max_eff = 25

  try:
    desired_vel = float(raw_input("desired velocity?"))
  except:
    print "invalid input, vel = -0.1"
    desired_vel =-0.1
  raw_input("Start logging data?")

  with open(trial_name+".txt", "a") as my_file:
    my_file.write(trial_name + "\t" + JOINTS[joint_idx] \
        + "\tmax_effort:\t" + str(max_eff) \
        + "\tdesired_velocity:\t" + str(desired_vel) +"\n")

    thread.start_new_thread(GetUserInput, ())
    
    joint.setMaxEffort(max_eff)
    joint.setController("velocity")
    joint.sendCommand(desired_vel)
    
    def stc(string):  #string with comma appended
      return str(string) + ","
    
    while not quit:
      out = ""
      m = joint.last_msg
      out = stc(m.time) + stc(m.position) + stc(m.velocity) + stc(m.commanded_effort)\
          + stc(m.measured_effort) + stc(m.filtered_velocity) +str(m.filtered_effort) + "\n"

      my_file.write(out)
      r.sleep()
  print "exiting!"

arg_inputs = len(sys.argv) >1

if(arg_inputs and sys.argv[1] == '-capture_data'):
  capture_data()
elif(arg_inputs and sys.argv[1] == '-test_collision'):
  check_collision()
else:
  print "USAGE: -caputure_data or -test_collision"
  sys.exit(2)

