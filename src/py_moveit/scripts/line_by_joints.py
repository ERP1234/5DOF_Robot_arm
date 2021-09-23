#!/usr/bin/env python


import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
## END_SUB_TUTORIAL

def all_close(goal, actual, tolerance):

  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class MoveGroupPythonIntefaceTutorial(object):

  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    ## BEGIN_SUB_TUTORIAL setup
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)

    ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
    ## the robot:
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
    ## to the world surrounding the robot:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the Panda
    ## arm so we set ``group_name = panda_arm``. If you are using a different robot,
    ## you should change this value to the name of your robot arm planning group.
    ## This interface can be used to plan and execute motions on the Panda:
    group_name = "arm"
    group = moveit_commander.MoveGroupCommander(group_name)
    planning_frame = group.get_planning_frame()

    ## We create a `DisplayTrajectory`_ publisher which is used later to publish
    ## trajectories for RViz to visualize:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)




    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame


  def go_to_joint_state(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group

    ## BEGIN_SUB_TUTORIAL plan_to_joint_state
    ##
    ## Planning to a Joint Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^^
    ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_ so the first
    ## thing we want to do is move it to a slightly better configuration.
    # We can get the joint values from the group and adjust some of the values:

    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = 0
    joint_goal[2] = 0
    joint_goal[3] = 0
    joint_goal[4] = 3.1
    group.go(joint_goal, wait=True)
    rospy.sleep(1)

    print "============ Press `Enter` to move to next point ..."
    raw_input()

    joint_goal = group.get_current_joint_values()
    joint_goal[0] = -0.57
    joint_goal[1] = -0.75
    joint_goal[2] = -0.94
    joint_goal[3] = -1.32
    joint_goal[4] = -0.57
    group.go(joint_goal, wait=True)
    rospy.sleep(3)

    print "============ Press `Enter` to move to next point ..."
    raw_input()

    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -0.5
    joint_goal[2] = -1.32
    joint_goal[3] = -1.19
    joint_goal[4] = 0
    group.go(joint_goal, wait=True)
    #rospy.sleep(1)

    #print "============ Press `Enter` to move to next point ..."
    #raw_input()

    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 0.57
    joint_goal[1] = -0.75
    joint_goal[2] = -0.94
    joint_goal[3] = -1.32
    joint_goal[4] = 0.57
    group.go(joint_goal, wait=True)

    print "============ Press `Enter` to move to next point ..."
    raw_input()

    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -0.5
    joint_goal[2] = -1.32
    joint_goal[3] = -1.19
    joint_goal[4] = 0
    group.go(joint_goal, wait=False)


    joint_goal = group.get_current_joint_values()
    joint_goal[0] = -0.57
    joint_goal[1] = -0.75
    joint_goal[2] = -0.94
    joint_goal[3] = -1.32
    joint_goal[4] = 0
    group.go(joint_goal, wait=True)
    rospy.sleep(3)








    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group


    # Calling ``stop()`` ensures that there is no residual movement
    group.stop()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)









def main():
  try:
    print "============ Press `Enter` to begin and launch  moveit_commander (press ctrl-d to exit) ..."
    raw_input()
    tutorial = MoveGroupPythonIntefaceTutorial()

    print "============ Press `Enter` to execute a movement using a joint state goal ..."
    raw_input()
    tutorial.go_to_joint_state()



    print "============ Python_moveit complete!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
