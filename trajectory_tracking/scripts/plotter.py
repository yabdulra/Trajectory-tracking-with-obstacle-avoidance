#!/usr/bin/env python

# import ROS Libraries
import rospy
from nav_msgs.msg import Odometry
from trajectory_tracking.msg import Traj
from geometry_msgs.msg import Pose

# import Matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time
import math

# global variables decalration 
global traj, robot_x, robot_y

traj = Traj()

xd = []
yd = [] 
xrobot = []
yrobot = []

# callback function for odom data
def odom_callbk(msg):
    global robot_x, robot_y

    robot_x = msg.pose.pose.position.x
    robot_y = msg.pose.pose.position.y

# callback function to get trajectory
def circular_traj_callbk(msg):
    global traj
    
    traj.pose.x = msg.pose.x
    traj.pose.y = msg.pose.y

# animate function to plot data
def animate1(i):
       
    xd.append(float(traj.pose.x))
    yd.append(float(traj.pose.y))
    ax1.plot(xd,yd, 'r',linestyle='solid',label="desired trajectory",lw=1)  

    xrobot.append(float(robot_x))
    yrobot.append(float(robot_y))
    ax1.plot(xrobot, yrobot, 'k', linestyle='dashdot', label='robot trajectory', lw=1)

    
    ax1.grid(True)
    ax1.legend()
    ax1.axis("equal")    # comment this line out if you'd like to set limits for x and y axis

    
if __name__ == '__main__':

    # initialization of ROS node
    rospy.init_node('plotter', anonymous=True) #make node
    rospy.Subscriber("/traj_topic", Traj, circular_traj_callbk, queue_size=10)
    rospy.Subscriber("/odom", Odometry, odom_callbk, queue_size=10)

    fig1 = plt.figure()
    ax1 = fig1.add_subplot(111)

    # uncomment to set axis limits
    # ax1.set_xlim([-17, 17])
    # ax1.set_ylim([-17, 17])
    
    time.sleep(0.1)
    ani1 = animation.FuncAnimation(fig1, animate1, interval=100)
    plt.show()
    rospy.spin()
