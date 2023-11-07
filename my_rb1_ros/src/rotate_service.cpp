#include "ros/duration.h"
#include "ros/node_handle.h"
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <my_rb1_ros/Rotate.h>

class RotateService
{
private:
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    ros::ServiceServer service_;
    const double ANGULAR_VELOCITY = 0.1;

public:
    RotateService(ros::NodeHandle& nh) : nh_(nh)
    {
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel",10);
        service_= nh_.advertiseService("/rotate_robot",&RotateService::rotateRobot,this);
    }

    bool rotateRobot(my_rb1_ros::Rotate::Request& req,
                     my_rb1_ros::Rotate::Response& res)
    {
        int degrees = req.degrees;
        double radians = degrees * M_PI /180.0;

        nav_msgs::Odometry::ConstPtr odom = ros::topic::waitForMessage<nav_msgs::Odometry>("/odom");
        double current_orientation = tf::getYaw(odom->pose.pose.orientation);

        double target_orientation = current_orientation + radians;

        geometry_msgs::Twist twist;
        twist.angular.z = ANGULAR_VELOCITY;

        cmd_vel_pub_.publish(twist);

        while (ros::ok())
        {
            odom = ros:: topic::waitForMessage<nav_msgs::Odometry>("/odom");
            current_orientation = tf::getYaw(odom->pose.pose.orientation);

            double orientation_diff = target_orientation - current_orientation;

            if (std::abs(orientation_diff) < 0.01)
                break;
            
            ros::Duration(0.1).sleep();
        }

        twist.angular.z = 0.0;
        cmd_vel_pub_.publish(twist);

        res.result = "Rotation completed successfully";

        return true;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc,argv,"rotation_server");
    ros::NodeHandle nh;

    RotateService rotate_service(nh);

    ros::spin();

    return 0;
}