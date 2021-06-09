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
    #eef.set_vel(0.1)
    #eef.set_accel(0.1)

    raw_input('Go to All Zeroes <enter>')
    rc.goto_all_zeros()
    #eef.goto_named_target('open')

    raw_input('Add cube <enter>')
    rc.add_object()

    raw_input('Line up with cube and lower <enter>')
    # pose [pos: x, y, z, axes:x y z w]
    #pose_lineup = [0,0.6,0.4,0,1,0,0]
    #rc.goto_Quant_Orient(pose_lineup)
    #pose_lower = [0,0.6,0.05,0,1,0,0]
    #rc.goto_Quant_Orient(pose_lower)
    pose_lower = [0,0.6,0.015,0,-1,0,0] # these values were guess/check
    # Need a way to detect z rotation of gripper
    # [0, 0.6, 0.015] --> cube position
        # Cube: w= 1, y= 0.6, z= 0.1
    # [0,1,0,0] --> Quaternion

    # Original:  [0,0.6,0.015,0,1,0,0] --> Quaternion = false
       # Try  1:  [0,0.6,0.015,0,1,1,0] --> no motion plan found
       # Try  2:  [0,0.6,0.015,1,1,0,0] --> no motion plan found
       # Try  3:  [0,0.6,0.015,0,1,0,1] --> no motion plan found
       # Try  4:  [0,0.6,0.015,0,1,1,1] --> no motion plan found
       # Try  5:  [0,0.6,0.015,1,1,1,1] --> no motion plan found
       # Try  6:  [0,0.6,0.015,0,0,1,1] --> no motion plan found, but moved linearly down from all-zeros
       # Try  7:  [0,0.6,0.015,0,1,1,1] --> no motion plan found, linearly downwards
       # Try  8:  [0,0.6,0.015,0,0,0,1] --> no motion plan found
       # Try  9:  [0,0.6,0.015,0,1,0,0] --> no motion plan found
       # Try 10:  [0,0.6,0.015,1,0,0,0] --> moved in same style as original
       # Try 11:  [0,0.6,0.015,0,1,0,_] --> TypeError: 'Pose' object does not support indexing
       # Try 12:  [0,1.0,0.015,0,1,0,0] --> No motion plan found
       # Try 13:  [0,0.9,0.015,0,1,0,0] --> No motion plan found
       # Try 14:  [0,0.7,0.015,0,1,0,0] --> Goes to edge of table
       # Try 15:  [0,0.6,0.100,0,1,0,0] --> Moves to correct location, but too high to pick up object
                                      # --> Quaterion = True, meaning within testing parameters according to all_close(goal, actual, tolerance) function
       # Try 16:  [0,0.6,0.015,0,0.5,0,0] --> no motion plan found, Quaterion = False
       # Try 17:  [0,0.6,0.015,0,0.1,0,0] --> no motion plan found Q=false
       # Try 18:  [0,0.6,0.015,0,-1,0,0] --> Successful, less of a rotation than original
       # Try 19:  [0,0.6,0.015-1,-1,0,0] --> No motion plan
       # Try 20:  [0,0.6,0.015,0,-1,-1,0] --> no motion plan found
       # Try 21:  [0,0.6,0.015,0,-1,0,-1] --> no motion plan found
       # Try 22:  [0,0.6,0.015,0,-1,-1,-1] --> no motion plan found




    # Need a way to detect z rotation
    # [0, 0.6, 0.015] --> cube position
    # [0,1,0,0] --> Quaternion
    print('Quanterion',rc.goto_Quant_Orient(pose_lower))
    rc.goto_Quant_Orient(pose_lower)
    #trying cartesian path (did not reach all the way to the cube)
    lower_plan = rc.plan_cartesian_lineup_path(-0.39)
    rc.execute_plan(lower_plan)

    raw_input('Grab the cube <enter>')
    
    eef.goto_named_target('close')
    joint_closed = eef.move_group.get_current_joint_values()
    joint_closed[0] = -0.007
    joint_closed[1] = 0.007
    eef.goto_joint_posn(joint_closed)
    rospy.sleep(3)

    rc.attach_object()
    rc.goto_all_zeros()
    rc.detach_object()
    rospy.sleep(3)
    rc.remove_object()

    raw_input('Go to Crouch position <enter>')
    rc.goto_crouch()
    

  except rospy.ROSInterruptException:
    exit()
  except KeyboardInterrupt:
    exit()

if __name__ == '__main__':
  main()