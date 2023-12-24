#!/usr/bin/env python3
from __future__ import print_function # Printing
import rospy # Python client library
import actionlib # ROS action library
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal # Controller messages
from std_msgs.msg import Float64 # 64-bit floating point numbers
from trajectory_msgs.msg import JointTrajectoryPoint # Robot trajectories
import sys
import time
from PyQt5.QtWidgets import QApplication, QMainWindow, QSlider, QVBoxLayout, QWidget, QLabel, QPushButton, QLineEdit
from PyQt5.QtCore import Qt
import threading
from std_msgs.msg import String




rospy.init_node('send_goal_to_arm', anonymous=True)
pub = rospy.Publisher('joint_angles', String, queue_size=10)
rate = rospy.Rate(1)  


def move_robot_arm(joint_values):
 
  
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
 


def ros(values):
    print(values)
    move_robot_arm(values)

def home():
    val = [0.00,0.00,-1.57,-1.57,1.57]

    move_robot_arm(val)


class SliderDemo(QMainWindow):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        self.setWindowTitle('Slider Demo')
        self.setGeometry(100, 100, 400, 400)

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)

        self.layout = QVBoxLayout()
        self.central_widget.setLayout(self.layout)

        self.sliders = []
        self.value_labels = []
        self.slider_names = ["BJ", "SJ", "EJ", "W1J", "W2J"]

        for i, slider_name in enumerate(self.slider_names):
            slider_name_input = QLineEdit()
            slider_name_input.setText(slider_name)
            self.layout.addWidget(slider_name_input)

            label = QLabel()
            self.layout.addWidget(label)

            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(-314)  # Minimum value multiplied by 100 for precision
            slider.setMaximum(314)   # Maximum value multiplied by 100 for precision
            slider.setValue(0)       # Set initial value to 0
            self.layout.addWidget(slider)

            value_label = QLabel()
            self.layout.addWidget(value_label)

            self.sliders.append(slider)
            self.value_labels.append(value_label)

            slider.valueChanged.connect(self.update_value_label)

        self.print_button = QPushButton('Move to Goal')
        self.layout.addWidget(self.print_button)
        self.print_button.clicked.connect(self.print_values)
        self.home_button = QPushButton('Home Position')
        self.layout.addWidget(self.home_button)
        self.print_button.clicked.connect(self.home)

    def update_value_label(self):
        for i, slider in enumerate(self.sliders):
            value = slider.value() / 100.0  # Divide by 100 to get the actual value
            self.value_labels[i].setText(f'Current Value: {value:.2f}')

    def print_values(self):
        values = [slider.value() / 100.0 for slider in self.sliders]  # Divide by 100 to get the actual value
        result = ros(values)

    def home(self):
       result1 = home()
    
    
def main():
    app = QApplication(sys.argv)
    window = SliderDemo()
    window.show()
    
    sys.exit(app.exec_())
   

if __name__ == '__main__':
    main()
