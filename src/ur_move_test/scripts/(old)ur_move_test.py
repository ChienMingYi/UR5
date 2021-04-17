#!/usr/bin/env python
# use moveit_commander (the Python MoveIt user interfaces )
from math import pi
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

class MoveGroupTutorial(object):
  """MoveGroupTutorial"""
  def __init__(self):
    super(MoveGroupTutorial, self).__init__()

    # First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_tutorial_ur5', anonymous=True)
 
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "manipulator" # See .srdf file to get available group names
    group = moveit_commander.MoveGroupCommander(group_name)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
    reference_frame = 'base_link'
    group.set_pose_reference_frame(reference_frame)
    planning_frame = group.get_planning_frame()
    eef_link = group.get_end_effector_link()
    group_names = robot.get_group_names()
    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names
    self.reference_frame = reference_frame


  def origin_pose(self):
    group = self.group
    pose_goal = geometry_msgs.msg.PoseStamped()
    pose_goal.header.frame_id = self.reference_frame
    pose_goal.header.stamp = rospy.Time.now()

    pose_goal.pose.orientation.w = 0.55
    pose_goal.pose.orientation.x = -0.42
    pose_goal.pose.orientation.y = 0.54
    pose_goal.pose.orientation.z = 0.46
    print('pose_goal.orientation:',pose_goal.pose.orientation)
    '''
    pose_goal.pose.position.x = 0.43
    pose_goal.pose.position.y = -0.3
    pose_goal.pose.position.z = 0.48
    print('pose_goal.position:',pose_goal.pose.position)
    '''
    group.set_start_state_to_current_state()

    group.set_pose_target(pose_goal, self.eef_link)
    
    plan = group.go(wait=True)
    '''
    group.stop()
    group.clear_pose_targets()

    current_pose = self.group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)
    '''

  def go_to_up_state(self):
    group = self.group

    joint_goal = group.get_current_joint_values()
    print(type(joint_goal), joint_goal)

    # joint goal is a list of 7 elements : (x,y,z,qx,qy,qz,qw) can be composed of pose_msg
    joint_goal[0] = -pi
    joint_goal[1] = -pi * 0.5
    joint_goal[2] = -pi * 0.5
    joint_goal[3] = -pi * 0.5
    joint_goal[4] = pi * 0.5
    joint_goal[5] = 0    

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    group.stop()

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def go_to_pose_goal(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group

    current_pose = group.get_current_pose().pose
    print("Current pose: ", current_pose)

    # We can plan a motion for this group to a desired pose for the end-effector:
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 0.7
    pose_goal.orientation.x = 0.0
    pose_goal.orientation.y = 0.7
    pose_goal.orientation.z = 0.0
    pose_goal.position.x = 0.43
    pose_goal.position.y = -0.3
    pose_goal.position.z = 0.3
    
    group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    group.clear_pose_targets()

    current_pose = group.get_current_pose().pose
    print("New current pose: ", current_pose)

    current_pose = self.group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)





def main():
  try:
    print("============ Press `Enter` to begin the tutorial by setting up the moveit_commander (press ctrl-d to exit) ...")
    raw_input()
    tutorial = MoveGroupTutorial()

    print("1")
    raw_input()
    tutorial.go_to_up_state()
    print("2")
    raw_input()
    tutorial.origin_pose()

    print("3")
    '''
    raw_input()
    tutorial.go_to_pose_goal()

    print("4")
    raw_input()
    tutorial.go_to_up_state()
    '''
    print("============ Python tutorial demo complete!")
  
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return
if __name__ == '__main__':
    main()