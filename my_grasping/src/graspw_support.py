#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, radians
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from motoman_msgs.srv import ReadSingleIO, WriteSingleIO

## Quaternion Tools
from tf.transformations import euler_from_quaternion, quaternion_from_euler

#from robot_support import moveManipulator
from any_position_grasp import *
def main():
  try:
    rc = moveManipulator('bot_mh5l')
    eef = moveManipulator('EEF')
    rc.set_vel(0.1)
    rc.set_accel(0.1)
    eef.set_vel(0.1)
    eef.set_accel(0.1)

    raw_input('Go to All Zeroes <enter>')
    rc.goto_all_zeros()
    eef.goto_named_target('open')

    raw_input('Add cube <enter>')
    rc.add_object()

    raw_input('Line up with cube and lower <enter>')
    # pose [pos: x, y, z, axes:x y z w]
    #pose_lineup = [0,0.6,0.4,0,1,0,0]
    #rc.goto_Quant_Orient(pose_lineup)
    #pose_lower = [0,0.6,0.05,0,1,0,0]
    #rc.goto_Quant_Orient(pose_lower)
    pose_lower = [0,0.6,-0.025,0,1,0,0]
    rc.goto_Quant_Orient(pose_lower)
    #trying cartesian path (did not reach all the way to the cube)
    #lower_plan = rc.plan_cartesian_lineup_path(-0.39)
    #rc.execute_plan(lower_plan)

    raw_input('Grab the cube <enter>')
    
    eef.goto_named_target('close')
    joint_closed = eef.move_group.get_current_joint_values()
    #joint_closed[0] = -0.007
    #oint_closed[1] = 0.007
    #eef.goto_joint_posn(joint_closed)
    rospy.sleep(3)

    rc.attach_object()
    rc.goto_all_zeros()
    rc.detach_object()
    rospy.sleep(10)
    rc.remove_object()

  except rospy.ROSInterruptException:
    exit()
  except KeyboardInterrupt:
    exit()

if __name__ == '__main__':
  main()