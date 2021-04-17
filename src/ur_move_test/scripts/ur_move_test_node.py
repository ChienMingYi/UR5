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
import numpy as np
from tf.transformations import quaternion_from_euler, euler_from_quaternion
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
    ee_link = group.get_end_effector_link()
    
    group.set_end_effector_link(ee_link)
    group.set_max_acceleration_scaling_factor(0.1)
    group.set_max_velocity_scaling_factor(0.1)
    

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.reference_frame = reference_frame
    self.end_effector_link = ee_link
    

  def origin_pose(self):
    group = self.group

    joint_goal = group.get_current_joint_values()
    print(type(joint_goal), joint_goal)

    # joint goal is a list of 7 elements : (x,y,z,qx,qy,qz,qw) can be composed of pose_msg
    joint_goal[0] = -pi * 0.5
    joint_goal[1] = -pi * 0.5
    joint_goal[2] = -pi * 0.5
    joint_goal[3] = -pi * 0.5
    joint_goal[4] = pi * 0.5
    joint_goal[5] = pi * 0.5    
    group.go(joint_goal, wait=True)
    print('pose :', group.get_current_pose().pose)
    # Calling ``stop()`` ensures that there is no residual movement
    group.stop()
    group.clear_pose_targets()
    current_joints = group.get_current_joint_values()
    print(current_joints)
    print('----------------')
    return all_close(joint_goal, current_joints, 0.01)

  def down(self):
    group = self.group
    pose_goal = group.get_current_pose().pose
    pose_goal.position.z -= 0.2

    quaternion = quaternion_from_euler(0,  np.radians(90.), 0)   
    pose_goal.orientation.x = quaternion[0]
    pose_goal.orientation.y = quaternion[1]
    pose_goal.orientation.z = quaternion[2]
    pose_goal.orientation.w = quaternion[3]

    group.go(pose_goal, wait=True)
    pose_msg = group.get_current_pose().pose.orientation
    euler_angle = euler_from_quaternion([pose_msg.x, pose_msg.y, pose_msg.z, pose_msg.w])
    print('----------------------------------')
    print(group.get_current_pose().pose)
    print(np.rad2deg(euler_angle))
    group.stop()
    

  def up(self):
    group = self.group
    pose_goal = group.get_current_pose().pose
    pose_goal.position.z += 0.2

    quaternion = quaternion_from_euler(0, np.radians(90.), 0)  
    pose_goal.orientation.x = quaternion[0]
    pose_goal.orientation.y = quaternion[1]
    pose_goal.orientation.z = quaternion[2]
    pose_goal.orientation.w = quaternion[3]

    group.go(pose_goal, wait=True)
    pose_msg = group.get_current_pose().pose.orientation
    euler_angle = euler_from_quaternion([pose_msg.x, pose_msg.y, pose_msg.z, pose_msg.w])
    print('----------------------------------')
    print(group.get_current_pose().pose)
    print(np.rad2deg(euler_angle))
    group.stop()

  def first_pose(self):
    group = self.group

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = 0.42
    pose_goal.position.y = -0.18
    pose_goal.position.z = 0.348
    
    #pose_goal.orientation = group.get_current_pose().pose.orientation
    
    quaternion = quaternion_from_euler(0,np.radians(90.), 0)   #pitch_angle,roll_angle, yaw_angle  #np.radians(90.)

    pose_goal.orientation.x = quaternion[0]
    pose_goal.orientation.y = quaternion[1]
    pose_goal.orientation.z = quaternion[2]
    pose_goal.orientation.w = quaternion[3]
    
    print('pose_goal.orientation:',pose_goal)

    group.set_pose_target(pose_goal, self.end_effector_link)
    #group.plan()
    plan = group.go(wait=True)
    pose_msg = group.get_current_pose().pose.orientation
    euler_angle = euler_from_quaternion([pose_msg.x, pose_msg.y, pose_msg.z, pose_msg.w])
    print(group.get_current_joint_values())
    print('----------------------------------')
    print(group.get_current_pose().pose)
    print(np.rad2deg(euler_angle))
    group.stop()
    group.clear_pose_targets()
  def second_pose(self):
    group = self.group


    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = 0.42
    pose_goal.position.y = 0.147
    pose_goal.position.z = 0.34
    
    quaternion = quaternion_from_euler(0,np.radians(90.), 0)   

    pose_goal.orientation.x = quaternion[0]
    pose_goal.orientation.y = quaternion[1]
    pose_goal.orientation.z = quaternion[2]
    pose_goal.orientation.w = quaternion[3]

    print('pose_goal.orientation:',pose_goal)

    group.set_pose_target(pose_goal, self.end_effector_link)
    plan = group.go(wait=True)
    pose_msg = group.get_current_pose().pose.orientation
    euler_angle = euler_from_quaternion([pose_msg.x, pose_msg.y, pose_msg.z, pose_msg.w])
    print(group.get_current_joint_values())
    print('----------------------------------')
    print(group.get_current_pose().pose)
    print(np.rad2deg(euler_angle))
    group.stop()
    group.clear_pose_targets()

  def third_pose(self):
    group = self.group
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.325
    pose_goal.position.z = 0.348
    
    quaternion = quaternion_from_euler(0, np.radians(90.), 0)   # pitch_angle,roll_angle, yaw_angle

    pose_goal.orientation.x = quaternion[0]
    pose_goal.orientation.y = quaternion[1]
    pose_goal.orientation.z = quaternion[2]
    pose_goal.orientation.w = quaternion[3]

    print('pose_goal.orientation:',pose_goal)

    group.set_pose_target(pose_goal, self.end_effector_link)
    plan = group.go(wait=True)
    pose_msg = group.get_current_pose().pose.orientation
    euler_angle = euler_from_quaternion([pose_msg.x, pose_msg.y, pose_msg.z, pose_msg.w])
    print('----------------------------------')
    print(group.get_current_pose().pose)
    print(np.rad2deg(euler_angle))
    group.stop()  


def main():
  
  try:
    print("============ Press `Enter` to begin the tutorial by setting up the moveit_commander (press ctrl-d to exit) ...")
    tutorial = MoveGroupTutorial()
    print('start')
    
    raw_input()
    print("origin_pose")
    tutorial.origin_pose()
    
    print('finish')
    raw_input()
    print('================================')
    print("first_pose")
    tutorial.first_pose()
    
    print('finish')
    raw_input()
    print('================================')
    print("down")
    tutorial.down()

    print('finish')
    raw_input()
    print('================================')
    print("up")
    tutorial.up()
    
    print('finish')
    raw_input()
    print('================================')
    print("second_pose")
    tutorial.second_pose()
    
    print('finish')
    raw_input()
    print('================================')
    print("down")
    tutorial.down()

    print('finish')
    raw_input()
    print('================================')
    print("up")
    tutorial.up()

    print('finish')
    raw_input()
    print('================================')
    print("third_pose")
    tutorial.third_pose()

    print('finish')
    raw_input()
    print('================================')
    print("down")
    tutorial.down()

    print('finish')
    raw_input()
    print('================================')
    print("up")
    tutorial.up()
    
    print('finish')
    raw_input()
    print('================================')
    print("origin_pose")
    tutorial.origin_pose()

    print("============ Python tutorial demo complete!")
  
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return
    
if __name__ == '__main__':
    main()