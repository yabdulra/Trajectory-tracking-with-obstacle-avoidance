#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>


class ChaseBall{
    private:
        ros::ServiceClient client;
        ros::Subscriber sub;
        ros::NodeHandle nh;
        std::vector<double> joints_last_position{ 0, 0 }, joints_current_position;
        ball_chaser::DriveToTarget srv;
    
    public:
        ChaseBall(){
            client = nh.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");
            sub = nh.subscribe("/camera/rgb/image_raw", 10, &ChaseBall::image_callback, this);
        }

        void drive_bot(double x, double z){
            ROS_INFO_STREAM("Driving bot to chase ball");

            srv.request.linear_x = x;
            srv.request.angular_z = z;

            if (!client.call(srv))
                ROS_ERROR("Failed to call service command_robot");
        }
        void image_callback(const sensor_msgs::Image& img){
            
            int column, white_pixel = 255;
            bool white_ball_detected = false;

            for (int i = 0; i < img.height * img.step; i+3) {
                if (img.data[i] == white_pixel && img.data[i+1] == white_pixel && img.data[i+2] == white_pixel) {
                    white_ball_detected = true;
                    column = (i / 3)%img.width;
                    break;
                }
            }

            if (!white_ball_detected){
                ROS_INFO_STREAM("No white ball detected");
                this->drive_bot(0.0, 0.0);
            }else{
                if(column < (img.width/3)){
                    ROS_INFO_STREAM("Ball to the left of the frame");
                    this->drive_bot(0.0, 0.6);
                }
                else if ((img.width/3) <= column && column <= 2*(img.width/3)){
                    ROS_INFO_STREAM("Ball at the center of the frame");
                    this->drive_bot(0.5, 0.0);
                }
                else if (column >= 2*(img.width/3)){
                    ROS_INFO_STREAM("Ball to the right of the frame");
                    this->drive_bot(0.5, -0.6);
                }
            }
        }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "process_image");
    
    ChaseBall chaseball;

    ros::spin();

    return 0;
}

