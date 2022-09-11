#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>
#include "trajectory_tracking/Traj.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <cmath>
#include <fstream>
#include <cstdlib>
#include <ctime>
#include <vector>
#include <algorithm>
using std::vector;

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
        double x_pos = 0.0, y_pos = 0.0, theta = 0.0, xs = 0.0, ys = 0.0;    

    public:
        Robot(){
            vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
            odom = nh.subscribe("odom", 10, &Robot::odom_callback, this);
            std::string node_name = ros::this_node::getName();
            nh.getParam(node_name + "/robot_radius", r_radius);
        }
        vector<int> sign_xs, sign_ys;
        double  r_radius, R_i;
        bool avoidance_suceeded = false;
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
        // takes in a vector of obstacles and compute the distance of each obstacle from the robot
        // returns a vector of an obstacle with the least distance to the robot
        vector<double> closest_obstacle(vector<vector<double> > &obstacles){
            vector<double> dist_to_robot(obstacles.size());
            vector<double> obstacle;
            for(auto i = 0; i != obstacles.size(); ++i ){
                dist_to_robot[i] = sqrt(pow((obstacles[i][0] - x_pos), 2) + pow((obstacles[i][1] - y_pos), 2));
            }

            int index = std::min_element(dist_to_robot.begin(), dist_to_robot.end()) - dist_to_robot.begin();
            obstacle.push_back(obstacles[index][0]);
            obstacle.push_back(obstacles[index][1]);
            obstacle.push_back(obstacles[index][2]);
            obstacle.push_back(dist_to_robot[index]);
            
            return obstacle;   // {x_obst, y_obst, r_obst, distance to robot}
        }

        void escape_critetion(){
            auto i = sign_xs.size(), j = sign_ys.size();
            // checks if the vectors are not empty, track sign changes and set correct boolean for
            // whether an obstacle if successfully avoided or not.
            if(i > 0 && j > 0){
                if((sign_xs[i-1] != sign_xs[0]) && (sign_ys[j-1] != sign_ys[0])){
                    avoidance_suceeded = true;
                }
                else{
                    avoidance_suceeded = false;
                }
            }
            (std::signbit(xs) == false)? sign_xs.push_back(1) : sign_xs.push_back(-1);
            (std::signbit(ys) == false)? sign_ys.push_back(1) : sign_ys.push_back(-1);
        }

        void avoidance_controller(vector<double> & obst){
            geometry_msgs::Twist vel;
            double k = 1.8;   //controller parameter 1.8
            double Rc = R_i - 0.2;  // radius of limit-cycle
            // define robot position with respect to the limit-cycle
            xs = x_pos - obst[0];
            ys = y_pos - obst[1];
            
            double xsdot = -ys + xs * (pow(Rc, 2) - pow(xs, 2) - pow(ys, 2));
            double ysdot = xs + ys * (pow(Rc, 2) - pow(xs, 2) - pow(ys, 2));
            double thetadot = atan2(ysdot, xsdot);
            double thetaerr = thetadot - theta;
            thetaerr = atan2(sin(thetaerr), cos(thetaerr));
            
            double xsdotdot = -ysdot - 2 * xs * (xs * xsdot + ys * ysdot) + xsdot * (pow(Rc, 2) - pow(xs, 2) - pow(ys, 2));
            double ysdotdot = xsdot - 2 * ys * (xs * xsdot + ys * ysdot) + ysdot * (pow(Rc, 2) - pow(xs, 2) - pow(ys, 2));
            double thetadotdot = (xsdot * ysdotdot - ysdot * xsdotdot) / (pow(xsdot, 2) + pow(ysdot, 2));

            vel.linear.x = 0.14;
            vel.angular.z = 1 * (thetadotdot + k * thetaerr);
            vel_pub.publish(vel);

            this->escape_critetion();
        }
};
// take in path to obstacles' txt file and return a vector of obstacles
// each obstacle is defined by its x and y coordinates and radius.
vector<vector<double> > obstacles(std::string path){
    double x, y, radius;
    vector<vector<double> > obsts;
    std::ifstream file(path);
    while(file >> x >> y >> radius){
        obsts.push_back({x, y, radius});
    }
    
    return obsts;
}


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
