#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

pcl::visualization::PCLVisualizer viewer("Point Cloud Viewer");  // 全局变量，用于显示点云图像

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    viewer.removePointCloud("cloud");  // 移除之前的点云数据
    viewer.addPointCloud(cloud, "cloud");  // 添加新的点云数据
    viewer.spinOnce();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud");
    ros::NodeHandle nh("~");

    // 加载bag文件
    rosbag::Bag bag;
    bag.open("/home/star/Desktop/ROS/ROS数据集/hello.bag/hello.bag", rosbag::bagmode::Read);

    // 创建点云话题的订阅者
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, pointCloudCallback);

    // 从bag文件中读取点云消息并显示
    rosbag::View view(bag, rosbag::TopicQuery("/velodyne_points"));

    ros::Rate rate(10);  // 控制循环的频率

    for (rosbag::MessageInstance const& msg : view)
    {
        sensor_msgs::PointCloud2::ConstPtr pc_msg = msg.instantiate<sensor_msgs::PointCloud2>();
        if (pc_msg != nullptr)
        {
            pointCloudCallback(pc_msg);
            rate.sleep();  // 添加延时
        }
    }

    // 关闭bag文件
    bag.close();

    return 0;
}
