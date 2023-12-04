#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "tf/tf.h"
#include <my_rb1_ros/Rotate.h>

class RotateService
{
private:
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    ros::ServiceServer service_;
    ros::Subscriber odom_sub_;
    const double ANGULAR_VELOCITY = 0.5;
    double current_yaw_;
    bool rotation_in_progress_;
    double roll;  // Declare roll as a member variable
    double pitch;  // Declare pitch as a member variable

public:
    RotateService(ros::NodeHandle& nh) : nh_(nh), rotation_in_progress_(false)
    {
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        service_ = nh_.advertiseService("/rotate_robot", &RotateService::rotateRobot, this);
        odom_sub_ = nh_.subscribe("/odom", 1, &RotateService::odomCallback, this);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(msg->pose.pose.orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(roll, pitch, current_yaw_);  // Update roll and pitch
    }

    bool rotateRobot(my_rb1_ros::Rotate::Request& req, my_rb1_ros::Rotate::Response& res)
    {
        if (rotation_in_progress_)
        {
            res.result = "Rotation already in progress...";
            return false;
        }

        int degrees = req.degrees;
        double radians = degrees * M_PI / 180.0;

        double target_yaw = current_yaw_ + radians;

        geometry_msgs::Twist twist;
        twist.angular.z = (degrees > 0) ? -ANGULAR_VELOCITY : ANGULAR_VELOCITY;

        cmd_vel_pub_.publish(twist);
        rotation_in_progress_ = true;

        ros::Time start_time = ros::Time::now();

        while (ros::ok())
        {
            ros::spinOnce();
            if (std::abs(current_yaw_ - target_yaw) < 0.05)  // Adjusted tolerance for stopping rotation
            {
                twist.angular.z = 0.0;
                cmd_vel_pub_.publish(twist);
                rotation_in_progress_ = false;
                res.result = "Rotation completed successfully";
                return true;
            }

            if ((ros::Time::now() - start_time).toSec() > 20.0)  // Timeout to prevent infinite rotation
            {
                twist.angular.z = 0.0;
                cmd_vel_pub_.publish(twist);
                rotation_in_progress_ = false;
                res.result = "Rotation timed out";
                return false;
            }
        }
        return false;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rotation_server");
    ros::NodeHandle nh;

    RotateService rotate_service(nh);
    ROS_INFO("Rotate Service is ready");
    
    ros::spin();

    return 0;
}