#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bag2image");
    ros::NodeHandle nh;

    if (argc != 3) {
        ROS_ERROR_STREAM("Usage: bag2image path_to_rosbag path_to_save_image");
        exit(1);
    }

    rosbag::Bag bag;
    bag.open(argv[1], rosbag::bagmode::Read);

    rosbag::View view(bag);

    std::string imageTopic = "/camera/image_raw";

    for (const rosbag::MessageInstance& msg : view)
    {
        if (msg.isType<sensor_msgs::Image>() && msg.getTopic() == imageTopic)
        {
            sensor_msgs::Image::ConstPtr image_msg = msg.instantiate<sensor_msgs::Image>();

            cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);

            cv::imwrite(argv[2], cv_image->image);

            break;
        }
    }

    bag.close();

    return 0;
}
