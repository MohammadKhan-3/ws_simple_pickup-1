#! /usr/bin/env python

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from math import pi, radians

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_grasping', anonymous=True)
rc = moveit_commander.RobotCommander()

robot_group = moveit_commander.MoveGroupCommander("bot_mh5l")

'''def point_down():
	joint_goal = rc.get_current_joint_values()
	joint_goal[0]=0
	joint_goal[1]=0
	joint_goal[2]=0
	joint_goal[3]=0
	joint_goal[4]=-radians(90)
	joint_goal[5]=0

	robot_group.go(joint_goal, wait=True)
	robot_group.stop()'''


# start position all zeros
robot_group.set_named_target("all-zeros")
robot_group.plan()
plan1 = robot_group.go(wait=True)

# Calling ``stop()`` ensures that there is no residual movement
robot_group.stop()
rospy.sleep(4)

#align with cube
pose_target = geometry_msgs.msg.Pose()
pose_target.position.x = 0
pose_target.position.y = 0.6
pose_target.position.z = 0.4

pose_target.orientation.x = 0
pose_target.orientation.y = 1
pose_target.orientation.z = 0
pose_target.orientation.w = 0

robot_group.set_pose_target(pose_target)
print(pose_target)
plan1 = robot_group.go()
robot_group.stop()
rospy.sleep(4)

#lower to cube
pose_target.position.z = 0.01
robot_group.set_pose_target(pose_target)
plan1 = robot_group.go()

#shutdown
rospy.sleep(5)
moveit_commander.roscpp_shutdown()
