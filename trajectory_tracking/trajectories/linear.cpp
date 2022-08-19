#include "ros/ros.h"
#include "trajectory_tracking/Traj.h"
#include <cmath>
#include <ctime>

class LinearTrajectory{
    private:
        ros::NodeHandle nh;
        ros::Publisher traj_pub;

    public:
        LinearTrajectory(){
            traj_pub = nh.advertise<trajectory_tracking::Traj>("/linear", 10); // define publisher topic 
        }
        double period;
        void publish_trajectory(double t){
            trajectory_tracking::Traj traj;

            double x_start, y_start, gradient;  // declare line parameters
            // get node name
            std::string node_name = ros::this_node::getName();  
            //get line parameters
            nh.getParam(node_name + "/x_start", x_start);
            nh.getParam(node_name + "/y_start", y_start);
            nh.getParam(node_name + "/gradient", gradient);
            nh.getParam(node_name + "/period", period);


            double k = 6 * M_PI / period;  // define constant K = 6Pi/T
            // define equations for positions x and y
            traj.pose.x = x_start + (1/gradient) * (k * t - y_start);
            traj.pose.y = y_start + gradient * (k * t - x_start);
            // compute the first derivatives of the positions (velocity)
            double x_dot = (1/gradient) * k;
            double y_dot = gradient * k;
            // compute theta
            traj.pose.theta = atan2(y_dot, x_dot);
            // compute the second derivatives of the positions (acceleration)
            double x_dotdot = 0;
            double y_dotdot = 0; 
            // compute reference velocities
            traj.ref_vel.linear.x = sqrt(pow(x_dot, 2) + pow(y_dot, 2));
            traj.ref_vel.angular.z = (x_dot * y_dotdot + y_dot * x_dotdot) / (pow(x_dot, 2) + pow(y_dot, 2));
            // publish trajectory information
            traj_pub.publish(traj);
        }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "linear");
    
    LinearTrajectory linear_trajectory;

    ros::Rate loop_rate(10);
    while (ros::ok()){
        time_t begin, now;
        time(&begin);

        double t = 0;
        while(t <= linear_trajectory.period){
            linear_trajectory.publish_trajectory(t);
            t = time(&now) - begin;
            loop_rate.sleep();
        }
    }
    return 0;
}
