


#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>

int main(int argc, char** argv) {
    ros::init(argc, argv, "showgraph");
    ros::NodeHandle nh;

    // 打开bag文件
    rosbag::Bag bag;
    bag.open("/home/star/Desktop/ROS/ROS数据集/unknown.bag", rosbag::bagmode::Read);

    // 设置需要读取的话题
    std::vector<std::string> topics = {"/camera/color/image_raw", "/camera/depth/image_rect_raw"};

    // 创建一个消息视图，只读取指定话题的消息
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    // 循环遍历消息
    for (const rosbag::MessageInstance& msg : view) {
        // 获取消息的时间戳和话题名称
        ros::Time timestamp = msg.getTime();
        std::string topic = msg.getTopic();

        // 检查当前消息的话题并处理颜色相机和深度相机消息
        if (topic == "/camera/color/image_raw") {
            sensor_msgs::Image::ConstPtr color_msg = msg.instantiate<sensor_msgs::Image>();
            if (color_msg != nullptr) {
                // 将ROS图像消息转换为OpenCV格式
                cv_bridge::CvImagePtr cv_ptr;
                try {
                    cv_ptr = cv_bridge::toCvCopy(color_msg, sensor_msgs::image_encodings::BGR8);
                    // 在这里处理颜色相机图像数据
                    cv::imshow("Color Camera", cv_ptr->image);
                    cv::waitKey(1);
                } catch (cv_bridge::Exception& e) {
                    ROS_ERROR("cv_bridge exception: %s", e.what());
                }
            }
        } else if (topic == "/camera/depth/image_rect_raw") {
            sensor_msgs::Image::ConstPtr depth_msg = msg.instantiate<sensor_msgs::Image>();
            if (depth_msg != nullptr) {
                // 将ROS图像消息转换为OpenCV格式
                cv_bridge::CvImagePtr cv_ptr;
                try {
                    cv_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
                    // 在这里处理深度相机图像数据
                    cv::Mat depth_image_float;
                    cv_ptr->image.convertTo(depth_image_float, CV_32FC1, 0.001); // 转换深度值为米
                    cv::imshow("Depth Camera", depth_image_float);
                    cv::waitKey(1);
                } catch (cv_bridge::Exception& e) {
                    ROS_ERROR("cv_bridge exception: %s", e.what());
                }
            }
        }
    }

    // 关闭bag文件和OpenCV窗口
    bag.close();
    cv::destroyAllWindows();

    return 0;
}