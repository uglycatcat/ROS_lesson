#include "ros/ros.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "showdata");
    rosbag::Bag bag;
    bag.open("/home/star/Desktop/ROS/ROS数据集/all.bag/all.bag", rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(std::string("/odom"));
    topics.push_back(std::string("/imu/data_raw"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    for (const rosbag::MessageInstance &msg : view)
    {
        if (msg.getTopic() == "/odom")
        {
            nav_msgs::Odometry::ConstPtr odom_msg = msg.instantiate<nav_msgs::Odometry>();
            if (odom_msg != nullptr)
            {
                ROS_INFO("Odometry Data: Position=(%.2f, %.2f, %.2f), Orientation=(%.2f, %.2f, %.2f, %.2f)",
                         odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z,
                         odom_msg->pose.pose.orientation.x, odom_msg->pose.pose.orientation.y,
                         odom_msg->pose.pose.orientation.z, odom_msg->pose.pose.orientation.w);
            }
        }
        else if (msg.getTopic() == "/imu/data_raw")
        {
            sensor_msgs::Imu::ConstPtr imu_msg = msg.instantiate<sensor_msgs::Imu>();
            if (imu_msg != nullptr)
            {
                ROS_INFO("IMU Data: Acceleration=(%.2f, %.2f, %.2f), Angular Velocity=(%.2f, %.2f, %.2f)",
                         imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z,
                         imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);
            }
        }
    }

    bag.close();
    return 0;
}
