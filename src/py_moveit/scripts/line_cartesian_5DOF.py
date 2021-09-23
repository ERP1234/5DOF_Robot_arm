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
    #joint_goal[0] = -0.57
    #joint_goal[1] = -0.75
    #joint_goal[2] = -0.94
    #joint_goal[3] = 0.2
    #joint_goal[4] = 0
    joint_goal[0] = -0.57
    joint_goal[1] = -0.75
    joint_goal[2] = -0.94
    joint_goal[3] = -1.32
    joint_goal[4] = -0.57
    group.go(joint_goal, wait=True)
    rospy.sleep(3)

    group.stop()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_joints = self.group.get_current_joint_values()
    current_pose = self.group.get_current_pose().pose

    print current_pose

    return all_close(joint_goal, current_joints, 0.01)

  def plan_cartesian_path(self, scale=1):
      # Copy class variables to local variables to make the web tutorials more clear.
      # In practice, you should use the class variables directly unless you have a good
      # reason not to.
      group = self.group

      ## BEGIN_SUB_TUTORIAL plan_cartesian_path
      ##
      ## Cartesian Paths
      ## ^^^^^^^^^^^^^^^
      ## You can plan a Cartesian path directly by specifying a list of waypoints
      ## for the end-effector to go through:
      ##
      waypoints = []

      wpose = group.get_current_pose().pose
      #wpose.position.z -= scale * 0.1  # First move up (z)
      #wpose.position.x += scale * 0.2  # Third move sideways (y)
      #waypoints.append(copy.deepcopy(wpose))

      wpose.position.y += scale * 1 # and sideways (y)
      waypoints.append(copy.deepcopy(wpose))


      #wpose.position.x += scale * 0.2  # Third move sideways (y)
      #waypoints.append(copy.deepcopy(wpose))

      #wpose.position.z += scale * 0.3  # Second move forward/backwards in (x)
      #waypoints.append(copy.deepcopy(wpose))

      # We want the Cartesian path to be interpolated at a resolution of 1 cm
      # which is why we will specify 0.01 as the eef_step in Cartesian
      # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
      (plan, fraction) = group.compute_cartesian_path(
                                         waypoints,   # waypoints to follow
                                         0.01,        # eef_step
                                         0)         # jump_threshold

      # Note: We are just planning, not asking move_group to actually move the robot yet:
      return plan, fraction

      ## END_SUB_TUTORIAL

  def display_trajectory(self, plan):
      # Copy class variables to local variables to make the web tutorials more clear.
      # In practice, you should use the class variables directly unless you have a good
      # reason not to.
      robot = self.robot
      display_trajectory_publisher = self.display_trajectory_publisher

      ## BEGIN_SUB_TUTORIAL display_trajectory
      ##
      ## Displaying a Trajectory
      ## ^^^^^^^^^^^^^^^^^^^^^^^
      ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
      ## group.plan() method does this automatically so this is not that useful
      ## here (it just displays the same trajectory again):
      ##
      ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
      ## We populate the trajectory_start with our current robot state to copy over
      ## any AttachedCollisionObjects and add our plan to the trajectory.
      display_trajectory = moveit_msgs.msg.DisplayTrajectory()
      display_trajectory.trajectory_start = robot.get_current_state()
      display_trajectory.trajectory.append(plan)
      # Publish
      display_trajectory_publisher.publish(display_trajectory);



  def execute_plan(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        group = self.group

        ## BEGIN_SUB_TUTORIAL execute_plan
        ##
        ## Executing a Plan
        ## ^^^^^^^^^^^^^^^^
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        group.execute(plan, wait=True)

        ## **Note:** The robot's current joint state must be within some tolerance of the
        ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
        ## END_SUB_TUTORIAL









def main():
  try:
    print "============ Press `Enter` to begin and launch  moveit_commander (press ctrl-d to exit) ..."
    raw_input()
    tutorial = MoveGroupPythonIntefaceTutorial()

    print "============ Press `Enter` to execute a movement using a joint state goal ..."
    raw_input()
    tutorial.go_to_joint_state()

    print "============ Press `Enter` to plan and display a Cartesian path ..."
    raw_input()
    cartesian_plan, fraction = tutorial.plan_cartesian_path()

    print "============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ..."
    raw_input()
    tutorial.display_trajectory(cartesian_plan)

    print "============ Press `Enter` to execute a saved path ..."
    raw_input()
    tutorial.execute_plan(cartesian_plan)



    print "============ Python_moveit complete!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
