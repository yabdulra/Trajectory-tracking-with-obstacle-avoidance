#include "ros/ros.h"
#include "ros/package.h"
#include "trajectory_tracking/Traj.h"
#include <cmath>
#include <ctime>
#include <fstream>
#include <cstdlib>
#include <iostream>
#include <vector>

class CircularTrajectory{
    private:
        ros::NodeHandle nh;
        ros::Publisher traj_pub;
        
    public:
        CircularTrajectory(){
            traj_pub = nh.advertise<trajectory_tracking::Traj>("/circular", 10); // define publisher topic 
        }
        double center_x, center_y, radius;  // declare circle parameters
        double period;
        void publish_trajectory(double t){
            trajectory_tracking::Traj traj;

            // get node name
            std::string node_name = ros::this_node::getName();  
            //get circle parameters
            nh.getParam(node_name + "/center_x", center_x);
            nh.getParam(node_name + "/center_y", center_y);
            nh.getParam(node_name + "/radius", radius);
            nh.getParam(node_name + "/period", period);

            double k = 2 * M_PI / period;  // define constant K = 2Pi/T
            // define equations for positions x and y
            traj.pose.x = center_x + radius * cos(k * t);
            traj.pose.y = center_y + radius * sin(k * t);
            // compute the first derivatives of the positions (velocity)
            double x_dot = -k * radius * sin(k * t);
            double y_dot = k * radius * cos(k * t);
            // compute theta
            traj.pose.theta = atan2(y_dot, x_dot);
            // compute the second derivatives of the positions (acceleration)
            double x_dotdot = -pow(k, 2) * radius * cos(k * t);
            double y_dotdot = -pow(k, 2) * radius * sin(k * t); 
            // compute reference velocities
            traj.ref_vel.linear.x = sqrt(pow(x_dot, 2) + pow(y_dot, 2));
            traj.ref_vel.angular.z = (x_dot * y_dotdot + y_dot * x_dotdot) / (pow(x_dot, 2) + pow(y_dot, 2));
            // publish trajectory information
            traj_pub.publish(traj);
        }

        void random_obstacles(std::string path){
            std::vector<int> angles;
            srand(time(0));
            angles.push_back(rand()%(90-50) + 50);
            angles.push_back(rand()%(180-140) + 140);
            angles.push_back(rand()%(270-230) + 230);
            angles.push_back(rand()%(360-320) + 320);
            
            std::ofstream file;
            file.open(path);
            for(auto i = 0; i != angles.size(); ++i){
                radius = ((0.26) * rand()) / RAND_MAX + radius - 0.13;

                double x = center_x + radius * cos(angles[i]*M_PI/180);
                double y = center_y + radius * sin(angles[i]*M_PI/180);
                double r = ((0.22 - 0.15) * rand()) / RAND_MAX + 0.15;

                file << x << " " << y << " " << r << "\n";
            }
            file.close();
        }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "circular");
    ros::NodeHandle private_node_handle("~");

    std::string path = private_node_handle.param<std::string>("path", "default");

    CircularTrajectory circular_trajectory;

    ros::Rate loop_rate(10);
    
    while (ros::ok()){
        time_t begin, now;
        time(&begin);
        
        double t = 0;
        circular_trajectory.publish_trajectory(t);
        circular_trajectory.random_obstacles(path);
        while(t <= circular_trajectory.period){
            circular_trajectory.publish_trajectory(t);
            t = time(&now) - begin;
            loop_rate.sleep();
        }
    }
    return 0;
}
