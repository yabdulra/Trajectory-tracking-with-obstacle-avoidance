#include "ros/ros.h"
#include "trajectory_tracking/Traj.h"
#include <cmath>
#include <ctime>

class CircularTrajectory{
    private:
        ros::NodeHandle nh;
        ros::Publisher traj_pub;

    public:
        CircularTrajectory(){
            traj_pub = nh.advertise<trajectory_tracking::Traj>("/circular", 10); // define publisher topic 
        }
        double period;
        void publish_trajectory(double t){
            trajectory_tracking::Traj traj;

            double center_x, center_y, radius;  // declare circle parameters
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
};

int main(int argc, char** argv){
    ros::init(argc, argv, "circular");
    
    CircularTrajectory circular_trajectory;

    ros::Rate loop_rate(10);
    while (ros::ok()){
        time_t begin, now;
        time(&begin);

        double t = 0;
        while(t <= circular_trajectory.period){
            circular_trajectory.publish_trajectory(t);
            t = time(&now) - begin;
            loop_rate.sleep();
        }
    }
    return 0;
}
