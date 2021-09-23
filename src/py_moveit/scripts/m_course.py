#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("arm1")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)


group.set_goal_orientation_tolerance(100)
print group.get_current_pose(end_effector_link = "end_link")




x= -0.000658586826111
y= 0.463723997582
z= 0.960858085428

group.set_position_target([x,y,z],  end_effector_link = "end_link")


group.set_planning_time(10)
plan1 = group.plan()
group.go(wait=True)

rospy.sleep(5)

moveit_commander.roscpp_shutdown()
