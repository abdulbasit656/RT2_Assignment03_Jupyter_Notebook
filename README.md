[Research Track II](https://corsi.unige.it/en/off.f/2022/ins/60236)<br>
**Documentor:** [Basit Akram](https://github.com/abdulbasit656)<br>
[M.Sc Robotics Engineering](https://corsi.unige.it/corsi/10635)<br>
[University of Genoa (UniGe)](https://unige.it/en)<br>
**Supervisor:** [Prof. Carmine Tommaso Recchiuto](https://rubrica.unige.it/personale/UkNDWV1r)

# Jupyter Notebook User Interface for Mobile Robot Simulator 

## Introduction
Jupyter Notebook is an open source web application which is used to create and share documents that contain real-time code, equation, data visualizations, text, and so on. This Jupyetr Nootebook is designed to create the User Interface of my code. Also, this notebook is developed as an user interface for the the project Software Architecture for Mobile Robot Control. For this, Jupyter Notebook tool is used.

This user interface should be able to control the project Mobile Robote Simulator. Also, user can operates and change the robot's behvaiour depending on their choice. Robot's Behvaiour can be autonomous drive, manual drive using teleop, and manual drive using teleop and avoiding the collisions.

## Installation
To create Jupyter Interface, Jupyter Notebook tool is required on your system. To install Jupyter follow the steps given below:

```
pip3 install jupyter bqplot pyyaml ipywidgets
```

```
jupyter nbextension enable --py widgetsnbextension
```
To get started, all you need to do is open up your terminal application and go to a folder of your choice. Then run the below command:
```
jupyter notebook --allow-root
```

Below is the figure which shows Notebook Server.
![jupyter](https://user-images.githubusercontent.com/17598805/230768480-cb618e51-020c-4788-9d5c-5c556704e58e.PNG)

## Jupyter and ROS

Jupyter Notebooks may be used and integrated with ROS called as [Jupyter-Ros](https://jupyter-ros.readthedocs.io/en/latest/). As for the other libraries, we need to install some extensions: 
```
pip3 install jupyros
```
For the publishing, the package contains tools to automatically generate widgets from message definitions. 
```
import jupyros as jr
import rospy
from std_msgs.msg import String
rospy.init_node('jupyter_node')
jr.publish('/sometopic', String)
```
This results in a jupyter widget where one can insert the desired message in the text field. The form fields (jupyter widgets) are generated automatically from the message definition. If we use a a different message type, we will get different fields.

**ROS3D** communicates with ROS via websocket. This communication is configured through the jupyter widgets protocol, but you are also required to run the *“rosbridge websocket”* package in your ROS environment (or launch file). For this, you need to make sure that you have ros-noetic-rosbridge-suite and ros-noetic-tf2-webrepublisher. Thus, for this example, install:
```
apt-get install ros-noetic-rosbridge-suit
```
```
apt-get install ros-noetic-tf2-web-republisher
```
```
apt-get install ros-noetic-slam-gmapping
```
```
apt-get install ros-noetic-move-base
```
For non-Docker Image user execute the aforementioned command by adding ***sudo*** in front. 

## Description of the Jupyter Code

The Jupyter user interface node is a super easy node. This is used to control the robot's behaviour. This implementation will demonstrates that robot's behavior such as switching to the different modalities such as:

* Autonomously reach a x,y coordinate inserted by the user
* Letting the user drive the robot with the keyboard
* Letting the user drive the robot assisting them to avoid collisions 

Additionally, these modalities can also be managed by using this interface. 

First, imported the required libraries as given below:
```
import rospy
import time
import os
import tf
import sys
import actionlib
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import ipywidgets as widgets
import jupyros as jr
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_matrix
from matplotlib.animation import FuncAnimation
from matplotlib import pyplot as plt
from jupyros import ros3d
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from IPython.display import display
from IPython.display import clear_output
from ipywidgets import Button, Layout, ButtonStyle, GridBox, VBox, HBox
from std_msgs.msg import Float32, Int32
from rt2_first_assignment.srv import Cmd
from rt2_first_assignment.srv import Dist
from rt2_first_assignment.msg import reach
```

## Initializing ROS node
```
rospy.init_node('jupy')
# Service for selecting the mode
client_cmd = rospy.ServiceProxy("/cmd", Cmd)
# Service for the destination in auto drive mode
client_dist = rospy.ServiceProxy("/dist", Dist)
# publishing velocity 
vel_pub = jr.publish('cmd_vel', Twist)
```
## UI: implementing the buttons
Below is the button interface for autonomous reach and Manual and Assistive operation of the robot:

```
# buttons for selecting modes
b1 = Button(description = 'Auto_drive mode',
           layout = Layout(width = 'auto', align = "center", grid_area = 'b1'),
           style = ButtonStyle(button_color = 'blue'))

b2 = Button(description = 'Manual_drive mode',
           layout = Layout(width = 'auto', grid_area = 'b2'),
           style = ButtonStyle(button_color = 'blue'))

b3 = Button(description = 'Assisted_drive mode',
           layout = Layout(width = 'auto', grid_area = 'b3'),
           style = ButtonStyle(button_color = 'blue'))

b4 = Button(description = 'Cancel Goal',
           layout = Layout(width = 'auto', grid_area = 'b4'),
           style = ButtonStyle(button_color = 'Yellow'))

b5 = Button(description = 'Reset Position',
           layout = Layout(width = 'auto', grid_area = 'b5'),
           style = ButtonStyle(button_color = 'lightblue'))

b6 = Button(description = 'End Program',
           layout = Layout(width = 'auto', grid_area = 'b6'),
           style = ButtonStyle(button_color = 'red'))

b7 = Button(description = 'Send Goal',
           layout = Layout(width = 'auto', grid_area = 'b7'),
           style = ButtonStyle(button_color = 'GreenYellow'))


# buttons for manual mode
b8 = Button(description = 'Turn Left',
           layout = Layout(width = 'auto', grid_area = 'b8'),
           style = ButtonStyle(button_color = 'GreenYellow'))

b9 = Button(description = 'Move Forward',
           layout = Layout(width = 'auto', grid_area = 'b9'),
           style = ButtonStyle(button_color = 'GreenYellow'))

b10 = Button(description = 'Turn Right',
           layout = Layout(width = 'auto', grid_area = 'b10'),
           style = ButtonStyle(button_color = 'GreenYellow'))

b11 = Button(description = 'Move Backward',
           layout = Layout(width = 'auto', grid_area = 'b11'),
           style = ButtonStyle(button_color = 'GreenYellow'))

b12 = Button(description = 'Stop',
           layout = Layout(width = 'auto', grid_area = 'b12'),
           style = ButtonStyle(button_color = 'Yellow'))


# getting the destination in auto drive mode
decided_x = widgets.FloatText(
    value = 0,
    description = 'X goal:',
    disabled = False,
)

decided_y = widgets.FloatText(
    value = 0,
    description = 'Y goal:',
    disabled = False,
)
```
## Function related to robot movement
```
# Define vel msg
vel_msg = Twist()
# Publisher for the velocity  in　Manual Drive　and assisted_drive.
input_vel_pub = rospy.Publisher('/input_cmd_vel', Twist, queue_size = 100)
global cmd, pos_x, pos_y, vel_msg, vel
    
# the fuctions in each mode
def auto_drive(b):
    rospy.wait_for_service("/cmd")
    print("Auto Drive")
    cmd = 1
    res1 = client_cmd(cmd)

def manual_drive(b):
    rospy.wait_for_service("/cmd")
    print("Manual Drive")
    cmd = 2
    res1 = client_cmd(cmd)
     
def assisted_drive(b):
    rospy.wait_for_service("/cmd")
    print("Assisted Drive")
    cmd = 3
    res1 = client_cmd(cmd)
    
def cancel_goal(b):
    rospy.wait_for_service("/cmd")
    print("cancel the goal")
    cmd = 4
    res1 = client_cmd(cmd)
    
def reset_position(b):
    rospy.wait_for_service("/cmd")
    print("car's position is resetted")
    cmd = 5
    res1 = client_cmd(cmd)
    
def end_program(b):
    rospy.wait_for_service("/cmd")
    print("end program")
    cmd = 0
    res1 = client_cmd(cmd)
    
# the destenation is fixed in auto drive mode
def set_position(b):
    rospy.wait_for_service("/dist")
    global decided_x, decided_y
    pos_x = decided_x.value
    pos_y = decided_y.value
    res2 = client_dist(pos_x, pos_y)
    print("destination is sent")
    
# Publishes the velocity in　Manual Drive　and assisted_drive.
def move_forward(b):
    vel_msg.linear.x = 0.5
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    input_vel_pub.publish(vel_msg)

def turn_left(b):
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0.5
    input_vel_pub.publish(vel_msg)

def turn_right(b):
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = -0.5
    input_vel_pub.publish(vel_msg)
    
def move_backward(b):
    vel_msg.linear.x = -0.5
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    input_vel_pub.publish(vel_msg)
    
def stop(b):
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    input_vel_pub.publish(vel_msg)
    
# each functions is called when a button is clicked
b1.on_click(auto_drive)
b2.on_click(manual_drive)
b3.on_click(assisted_drive)
b4.on_click(cancel_goal)
b5.on_click(reset_position)
b6.on_click(end_program)
b7.on_click(set_position)
b8.on_click(turn_left)
b9.on_click(move_forward)
b10.on_click(turn_right)
b11.on_click(move_backward)
b12.on_click(stop)
```
## Choose the mode
![mode](https://user-images.githubusercontent.com/17598805/230768794-12a5113d-83d5-4d1c-b251-aa2f7be39b5d.PNG)

## Select (x,y) goal in auto drive mode
Select (x,y) goal position and send the goal
![autogoal](https://user-images.githubusercontent.com/17598805/230768839-8a9da0dc-5447-45d3-be6d-98d0acb06bdf.PNG)

## Choose direction in manual mode and assisted mode
![manualAssist](https://user-images.githubusercontent.com/17598805/230768906-7772e6f2-d713-43e6-a10d-b4aa24b13db8.PNG)


## Robot's Odometry & Laser Scanner
This part will demonstrates the graphical representation of the robot's behavior during the Simulation.

* The first graph will represents the position described by the odometry topic of the robot. The graph will show the position of the robot in space with red dots. 
* The second graph will represents the laser scan graph representation. 
* The third graph will represents the either goal is reached or not.

Below is the function shows the plotting of the data of the robot's odometery and laser scanner.
```
class Odom_Visualiser:
    def __init__(self):
        self.fig_odom, self.ax1 = plt.subplots()
        plt.grid(True)
        self.ln1, = self.ax1.plot([], [], 'ro')
        self.x_data, self.y_data = [], []
    def plot_init(self):
        self.ax1.set_title("Robot Odometry", fontsize = 20, fontweight = 'bold')
        self.ax1.set_xlabel("X [m]", fontsize = 10, fontweight = "bold")
        self.ax1.set_ylabel("Y [m]", fontsize = 10, fontweight = "bold")
        self.ax1.set_xlim(-20, 20)
        self.ax1.set_ylim(-20, 20)
        return self.ln1
    
    def odom_callback(self, msg):
        self.y_data.append(msg.pose.pose.position.y)
        self.x_data.append(msg.pose.pose.position.x)         
        
    def update_plot(self, frame):
        self.ln1.set_data(self.x_data, self.y_data)
        return self.ln1

class Laser_Visualiser:
    def __init__(self):
        self.fig_laser = plt.figure(figsize = (7, 7))
        self.ax2 = plt.subplot(111, polar = True)
        plt.grid(True)
        self.ax2.set_thetalim(-np.pi/2, np.pi/2)
        self.ax2.set_rmax(20)
        self.ax2.set_theta_zero_location("N")
        self.ln2, = self.ax2.plot([], [], 'b-')
        self.laser_dist = []
        self.laser_ang = []
        
    def plot_init(self):
        self.ax2.set_title("Laser Scanner Data", fontsize = 20, fontweight = "bold")
        self.ax2.set_xlabel("Distance [m]", fontsize = 10, fontweight = "bold")
        return self.ln2
    
    def laser_callback(self, msg):
        global laser_ang
        self.laser_dist = msg.ranges
        self.laser_ang = np.arange(msg.angle_min, msg.angle_max+msg.angle_increment, msg.angle_increment)
        
    def update_plot(self, frame):
        global laser_ang
        self.ax2.set_rmax(20)
        self.ln2.set_data(self.laser_ang, self.laser_dist)
        return self.ln2
    
class Goal_Visualiser:
    def __init__(self):
        self.fig_goal, self.ax3 = plt.subplots()
        plt.grid(False)
        self.reached = 0
        self.unreached = 0
        self.ln3 = plt.bar(['Reached', 'Unreached'], [self.reached, self.unreached], color = ['red', 'green'])
        
    def plot_init(self):
        self.ax3.set_title("Goals are reached or not", fontsize = 20, fontweight = "bold")
        self.ax3.set_xlabel("Goal success", fontsize = 10, fontweight = "bold")
        self.ax3.set_ylabel("Numbers", fontsize = 10, fontweight = "bold")
        self.ax3.set_ylim(0, 16)
        self.ax3.set_yticks(np.arange(0, 16, 2))
        return self.ln3
    
    def goal_callback(self, msg):
        self.reached = msg.reach
        self.unreached = msg.unreach     
        
    def update_plot(self, frame):
        self.ln3 = plt.bar(['Reached', 'Unreached'], [self.reached, self.unreached], color = ['red', 'green'])
        return self.ln3
```
### Graph 01
![odometry](https://user-images.githubusercontent.com/17598805/230769375-b8356826-f827-40a6-b748-bb00132e014b.PNG)


### Graph 02
![laserscan](https://user-images.githubusercontent.com/17598805/230769095-e70df3df-5396-41e0-89f5-cd12d8d07d7a.PNG)

### Graph 03
![reached](https://user-images.githubusercontent.com/17598805/230769906-eee8619d-6f5d-465c-a4f6-8fa64d857e7d.PNG)
