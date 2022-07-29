# Where Am I
This project form part of the Udacity's Robotics Software Engineer Nanodegree. It is aimed at demonstrating how to localize a mobile robot in ROS using the Adaptive Monte Carlo Localization (AMCL). The repository consist of the following:

* A Gazebo world and a mobile robot coloned from [this repo](https://github.com/yabdulra/Go-Chase-It.git).
* ROS packages: [map_server](http://wiki.ros.org/map_server), [amcl](http://wiki.ros.org/amcl), and [move_base](http://wiki.ros.org/move_base).
* The `maps` directory containing pgm map file generated using the [pgm_map_creator](https://github.com/udacity/pgm_map_creator.git).

## Prerequisites
* ROS and Gazebo running on Linux.
* CMake and gcc/g++.
* Install dependencies using the following commands:
    ```
    $ sudo apt-get update
    $ sudo apt-get upgrade -y
    $ sudo apt-get install ros-<your distro>-map-server
    $ sudo apt-get install ros-<your distro>-amcl
    $ sudo apt-get install ros-<your distro>-move-base
    ```

## Build
* Clone the repo to the src folder of your catkin workspace
    ```
    $ git clone https://github.com/yabdulra/Where-Am-I.git
    ```

    Follow [this](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) guide to create a catkin workspace if you dont have one.
* Within the same src folder, clone the teleop package
    ```
    $ git clone https://github.com/ros-teleop/teleop_twist_keyboard
    ```
* Change directory to `catkin_ws` and build.
    ```
    $ cd ..
    $ catkin_make
    ```

## Launch
* Source your workspace and launch the simulation world.
    ```
    $ source devel/setup.bash
    $ roslaunch my_robot world.launch
    ```

* In a new terminal, source your workspace and launch the amcl.
    ```
    $ source devel/setup.bash
    $ roslaunch my_robot amcl.launch
    ```
    This will launch the map_server, move_base, and the amcl.
* In a new terminal. source your workspace and run the `teleop` node.
    ```
    $ source devel/setup.bash
    $ rosrun teleop_twist_keyboard teleop_twist_keyboard.py
    ```
    Use this terminal to move the robot around. You will observe on rviz how AMCL is updating the particles as the robot pose is updated.

    ![1](https://user-images.githubusercontent.com/61895971/181671583-95b88d42-7539-4043-9105-3fe0e1fb2332.png)

    
    You should consider checking the `sample_screenshots` folder for views of the robot localizing itself at different positions on the map.