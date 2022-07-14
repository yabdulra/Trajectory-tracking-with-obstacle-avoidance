#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"


class DriveBot{
    private:
        ros::NodeHandle nh;
        ros::ServiceServer service;
        ros::Publisher vel_pub;
    
    public:
        DriveBot(){
            service = nh.advertiseService("/ball_chaser/command_robot", &DriveBot::handle_motor_vel_request, this);
            vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        }

        bool handle_motor_vel_request(ball_chaser::DriveToTarget::Request& req, ball_chaser::DriveToTarget::Response& res){
            ROS_INFO("DriveToTargetRequest received - x:%1.2f, z:%1.2f", (float)req.linear_x, (float)req.angular_z);

            geometry_msgs::Twist vel;
            ROS_INFO("Ready to send velocity commands");

            vel.linear.x = req.linear_x;
            vel.angular.z = req.angular_z;
            vel_pub.publish(vel);

            res.msg_feedback = "Velocities set - x: " + std::to_string(req.linear_x) + " , z: " + std::to_string(req.angular_z);
            ROS_INFO_STREAM(res.msg_feedback);

            return true;
        }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "drive_bot");

    DriveBot drivebot;

    ros::spin();

    return 0;
}
