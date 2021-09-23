joint_goal = group.get_current_joint_values()
joint_goal[0] = 0.57
joint_goal[1] = -0.75
joint_goal[2] = -0.94
joint_goal[3] = -1.32
joint_goal[4] = 0.57
group.go(joint_goal, wait=True)
#rospy.sleep(5)



joint_goal = group.get_current_joint_values()
joint_goal[0] = 0
joint_goal[1] = -0.5
joint_goal[2] = -1.32
joint_goal[3] = -1.19
joint_goal[4] = 0
group.go(joint_goal, wait=True)
#rospy.sleep(1)


joint_goal = group.get_current_joint_values()
joint_goal[0] = -0.57
joint_goal[1] = -0.75
joint_goal[2] = -0.94
joint_goal[3] = -1.32
joint_goal[4] = -0.57
group.go(joint_goal, wait=True)
rospy.sleep(3)
