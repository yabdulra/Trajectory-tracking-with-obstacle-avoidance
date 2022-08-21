#include <ros/ros.h>
#include <tf/tf.h>
#include "trajectory_tracking/Traj.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <cmath>
#include <cstdlib>
#include <ctime>

class Trajectory{
    private:
        ros::NodeHandle n;
        ros::Subscriber traj;
    public:
        Trajectory(){
            traj = n.subscribe("/traj_topic", 10, &Trajectory::traj_callback, this);
        }
        double x_traj, y_traj, theta_traj, v_r, w_r;

        void traj_callback(const trajectory_tracking::Traj &msg){
            // get trajectory info
            x_traj = msg.pose.x;
            y_traj = msg.pose.y;
            theta_traj = msg.pose.theta;
            v_r = msg.ref_vel.linear.x;
            w_r = msg.ref_vel.angular.z;
        }
};

class Robot{
    private:
        ros::NodeHandle nh;
        ros::Publisher vel_pub;
        ros::Subscriber odom;
        double x_pos = 0.0, y_pos = 0.0, theta = 0.0;

    public:
        Robot(){
            vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
            odom = nh.subscribe("odom", 10, &Robot::odom_callback, this);
        }
        
        void odom_callback(const nav_msgs::Odometry::ConstPtr &msg){
            // get robot's x and y position
            x_pos = msg->pose.pose.position.x;
            y_pos = msg->pose.pose.position.y;
            // convert theta from Quaternion to Euler and get robot's yaw
            tf::Quaternion quat;
            tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
            double roll, pitch, yaw;
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
            theta = yaw;
        }

        void tracking_controller(const Trajectory &t){
            geometry_msgs::Twist vel;
            // error dynamics
            double e_x = (t.x_traj - x_pos) * cos(theta) + (t.y_traj - y_pos) * sin(theta);
            double e_y = -(t.x_traj - x_pos) * sin(theta) + (t.y_traj - y_pos) * cos(theta);
            double e_theta = t.theta_traj - theta;
            e_theta = atan2(sin(e_theta), cos(e_theta));  // normalize theta between -PI and PI
            // controller parameters
            double ep = 0.9, b = 18;
            double a = sqrt(pow(t.v_r, 2) + pow(t.w_r, 2));
            double kx = 2 * ep * a, ky = b * abs(t.v_r), kz = kx;
            // compute control inputs
            vel.linear.x = t.v_r * cos(e_theta) + kx * e_x;
            vel.angular.z = t.w_r + (ky * e_y * sin(e_theta) / e_theta) + kx * e_theta;
            vel_pub.publish(vel);
        }
};


int main(int argc, char** argv){
    ros::init(argc, argv, "robot");
    Trajectory trajectory;
    Robot robot;
    ros::Rate loop_rate(10);

    while(ros::ok()){
        robot.tracking_controller(trajectory);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
