#!/usr/bin/env python3

# Import the necessary libraries
from __future__ import print_function # Printing
import rospy # Python client library
import actionlib # ROS action library
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal # Controller messages
from std_msgs.msg import Float64 # 64-bit floating point numbers
from trajectory_msgs.msg import JointTrajectoryPoint # Robot trajectories
import sys
import time
def move_robot_arm(joint_values):
  """
  Function to move the robot arm to desired joint angles.
  :param: joint_values A list of desired angles for the joints of a robot arm 
  """
  
  # Create the SimpleActionClient, passing the type of the action to the constructor
  arm_client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
 
  # Wait for the server to start up and start listening for goals.
  arm_client.wait_for_server()
     
  # Create a new goal to send to the Action Server
  arm_goal = FollowJointTrajectoryGoal()
 
  # Store the names of each joint of the robot arm
  arm_goal.trajectory.joint_names = ['BJ', 'SJ','EJ' ,'W1J', 'W2J']
   
  # Create a trajectory point   
  point = JointTrajectoryPoint()
 
  # Store the desired joint values
  point.positions = joint_values
 
  # Set the time it should in seconds take to move the arm to the desired joint angles
  point.time_from_start = rospy.Duration(0.5)
 
  # Add the desired joint values to the goal
  arm_goal.trajectory.points.append(point)
 
  # Define timeout values
  exec_timeout = rospy.Duration(10)
  prmpt_timeout = rospy.Duration(5)
 
  # Send a goal to the ActionServer and wait for the server to finish performing the action
  arm_client.send_goal_and_wait(arm_goal, exec_timeout, prmpt_timeout)
 
 
if __name__ == '__main__':
  """
  Main method.
  """
  try:
    # Initialize a rospy node so that the SimpleActionClient can
    # publish and subscribe over ROS.
    rospy.init_node('send_goal_to_arm_py')
    
    ans = input(" You want to see some movess!!!       ")

 
    # Move the joints of the robot arm to the desired angles in radians

    if ans == "yes":

      for i in range(2):
        for  i in range(4):
          move_robot_arm([-1.16, -1.08, 0.37, -0.55, 1.16])
          rospy.logwarn(" Ohhh baby calmm downn...")
          time.sleep(1)
          move_robot_arm([0, 0, 0, 0, 0])
          rospy.logwarn(" Ohhh calm downn baby calm down...")
          time.sleep(1)
    
        for i in range(5):
          move_robot_arm([-1.16, -1.08, 0.37, -0.55, 1.16])
          rospy.logwarn(" Ohhh baby calmm downn...")
          time.sleep(1)
          move_robot_arm([-0.33, -1.08, 0.37, -0.55, 1.16])
          rospy.logwarn(" Ohhh calm downn baby calm down...")
          time.sleep(1)
    

  

 
    print("want more!!!!!")
     
  except rospy.ROSInterruptException:
    print("Program interrupted before completion.", file=sys.stderr)
