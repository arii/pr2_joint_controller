#!/usr/bin/python
import os,sys

def main():
  run = "rosrun pr2_controller_manager pr2_controller_manager stop "
  os.system(run + "r_arm_controller")
  os.system(run + "l_arm_controller")

main()
