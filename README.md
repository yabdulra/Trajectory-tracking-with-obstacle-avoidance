# Trajectory tracking with obstacle avoidance

## 1. Overview
This ROS package uses the tracking controller proposed in this [paper](https://www.sciencedirect.com/science/article/abs/pii/S0005109897000551) coupled with a new variant of the limit-cycle obstacle avoidance strategy to implement trajectory tracking with obstacle avoidance for nonholonomic constrained mobile robots. It uses hierarchical action selection to activate the avoidance controller when a disturbing obstacle is detected, and then switch to tracking controller in an obstacle free location.

## 2. Build
* Clone the repo to the src folder of your catkin workspace
    ```
    $ git clone https://github.com/yabdulra/Trajectory-tracking-with-obstacle-avoidance.git
    ```
    To use the tracking controller without obstacle avoidance, clone the repo using:
    ```
    $ git clone -b v1.0 https://github.com/yabdulra/Trajectory-tracking-with-obstacle-avoidance.git
    ```

* Change directory to `catkin_ws` and build.
    ```
    $ cd ..
    $ catkin_make
    ```

## 3. Configuration
A default launch file for the tracking controller, reference trajectory, and the plotter can be found in the `trajectory_tracking` package directory. The launch file contains the following configurable parameters:  
* path: lets you define where to store obstacles' data. The avoidance controller in the robot node uses this data to determine the obstacle the robot must avoid, while the plotter node uses it to plot a representation of the obstacle.
* center_x, center_y, radius, period: used to define parameters for the circular trajectory.
* robot_radius: the avoidance controller uses this along with other predefined parameters to define safe margin for collision avoidance.

  Below is the sample configuration which can be modified in the launch file.
  ```
  <launch>
    <arg name="path" value="$(find trajectory_tracking)/src/obstacles.txt" />

    <node ns="circular" name="plotter" type="plotter.py" pkg="trajectory_tracking" output="screen">
      <remap from="/traj_topic" to="/circular" />
    </node>
    <node name="circular" type="circular" pkg="trajectory_tracking" output="screen">
      <param name="path" value="$(arg path)" />

      <rosparam>
        center_x: 0
        center_y: 0
        radius: 3
        period: 150
      </rosparam>
    </node>

    <node name="robot" type="tracking_controller" pkg="trajectory_tracking" output="screen">
      <param name="path" value="$(arg path)" />
      <remap from="/traj_topic" to="/circular" />
      <rosparam>
        robot_radius: 0.089
      </rosparam>
    </node>
  </launch>
  ```

## 4. Nodes
* robot:
  - subscribed topics:
    * traj_topic (trajectory_tracking/Traj): The custom message contains the 2D pose of the trajectory and a referecne linear and angular velocities. This topic is remapped to subscribe to desired trajectory topic (circular, linear, 8_shape) of the same message type. The tracking controller uses this message to calculate appropriate control inputs to the robot.
    * odom ([nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html)): Contains robot's pose data used by both controllers to calculate appropriate control inputs to the robot.
  - published topics:
    * cmd_vel ([geometry_msgs/Twist](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html)): the node uses this topic to publish velocity commands to drive the robot.
* circular:
  - published topics:
    * circular (trajectory_tracking/Traj): contains pose and reference velocity of the circular trajectory.
* plotter:
  - sbscribed topics:
    * odom ([nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html)): The node uses this data to plot the robot's trajectory.
    * traj_topic (trajectory_tracking/Traj): The node uses thid data to plot the reference trajectory. This topic is remapped to subscribe to desired trajectory topic (circular, linear, 8_shape) of the same message type.
## 5. Launch
* Source your workspace and launch your robot.

* In a new terminal, source your workspace and launch the trajectory_tracking package.
    ```
    $ source devel/setup.bash
    $ roslaunch trajectory_tracking main.launch
    ```

## 6. Visualization
Once the nodes are launched, you can visualize the simulated obstacles on the plotter as well as the robot's and reference trajectories. You can see the robot deviating from the referecne trajectory to avoid any obstacle that can obstruct its motion, and then switching back to tracking the trajectory when the obstacle is completely avoided.  
Below are sample views of a robot tracking a circular trajectory of radius 3m centered at the origin. The plots depict the robot avoiding some randomly generated disturbing obstacles.
<table style="width:75%">
  <tr> 
    <th><p>
           https://user-images.githubusercontent.com/61895971/189815304-b5619fe8-102d-461f-9c6f-90f0ea4cd694.png
      </p>
    </th>
    <th><p>
           https://user-images.githubusercontent.com/61895971/189815269-f34dffce-b651-4786-b64f-1c5d022ba8d1.png
        </p>
    </th>
  </tr>
</table>
