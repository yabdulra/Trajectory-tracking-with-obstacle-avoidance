#!/usr/bin/env python

# import ROS Libraries
import rospy
from nav_msgs.msg import Odometry
from trajectory_tracking.msg import Traj

import rospkg

# import Matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time
import math
import random

# global variables decalration 
global traj, robot_x, robot_y

traj = Traj()

xd = []
yd = [] 
xrobot = []
yrobot = []
colors = [
    (0.078, 0.804, 0.784, 1), (0.000, 0.412, 0.573, 1), (0.671, 0.718, 0.718, 1),
    (0.463, 0.365, 0.412, 1), (1.000, 0.580, 0.439, 1), (0.424, 0.478, 0.537, 1),
    (0.686, 0.255, 0.329, 1), (0.945, 0.353, 0.133, 1), (0.431, 0.188, 0.294, 1),
    (0.635, 0.871, 0.817, 1)
]

def obstacles(path):
    global obsts
    with open(path + '/src/obstacles.txt') as f:
        obsts = f.readlines()
    obsts = [obst.strip() for obst in obsts]
    for i in range(len(obsts)):
        obsts[i] = [float(x) for x in obsts[i].split()]

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
    
    for obst in obsts:
        ax1.add_artist( plt.Circle((obst[0], obst[1]), obst[2], color=colors[random.randint(0, 9)], alpha=0.9))
    
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

    rospack = rospkg.RosPack()
    path = rospack.get_path('trajectory_tracking')
    obstacles(path)

    # uncomment to set axis limits
    # ax1.set_xlim([-17, 17])
    # ax1.set_ylim([-17, 17])
    
    time.sleep(0.1)
    ani1 = animation.FuncAnimation(fig1, animate1, interval=100)
    plt.show()
    rospy.spin()
