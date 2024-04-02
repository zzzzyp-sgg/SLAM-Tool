#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <fstream>
#include <sstream>

/* imu数据转bag，bag包以IMU数据为基础(因为通常imu数据是时间最长的) */
void imu2bag(rosbag::Bag &bag, const std::string imuFile, const std::string outBag, int gpsWeek)
{
    std::ifstream file(imuFile);
    if (!file.is_open())
    {
        ROS_ERROR_STREAM("Failed to open file!");
        exit(1);
    }

    bag.open(outBag, rosbag::bagmode::Write);

    std::string line;
    std::getline(file, line);   // 跳过第一行

    while (std::getline(file, line))
    {
        // 将每行数据分割为各个字段
        std::istringstream iss(line);
        double time, gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z;
        if (!(iss >> time >> gyro_x >> gyro_y >> gyro_z >> accel_x >> accel_y >> accel_z))
        {
            ROS_WARN_STREAM("Failed to parse line: " << line);
            continue;
        }

        // 创建IMU消息
        sensor_msgs::Imu imu_msg;
        // 315964800是GPS起始时间和计算机起始时间的一个固定差值
        time = time + 315964800 + 604800 * gpsWeek - 8 * 3600;
        imu_msg.header.stamp = ros::Time(time);
        imu_msg.angular_velocity.x = gyro_x;
        imu_msg.angular_velocity.y = gyro_y;
        imu_msg.angular_velocity.z = gyro_z;
        imu_msg.linear_acceleration.x = accel_x;
        imu_msg.linear_acceleration.y = accel_y;
        imu_msg.linear_acceleration.z = accel_z;

        // 写入ROSbag文件
        bag.write("/imu0", ros::Time(time), imu_msg);
    }

    bag.close();
    file.close();
    std::cout << "imu data convert finished!" << std::endl;
}

/* image转bag */
void image2bag(rosbag::Bag &bag, const std::string &strPathToImage, const std::string outBag, int gpsWeek, std::string LorR)
{
    std::ifstream fTime;
    std::string strPathToTime = strPathToImage + "/timestamps.txt";
    fTime.open(strPathToTime);
    if (!fTime.is_open())
    {
        ROS_ERROR_STREAM("Failed to open timestamp file!");
        exit(1);
    }

    // 保存时间戳
    std::vector<double> vTimeStamp;
    while(!fTime.eof())
    {
        std::string s;
        getline(fTime, s);
        if (!s.empty()) {
            std::stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimeStamp.push_back(t);
        }
    }
    fTime.close();

    // 开始转换图片
    bag.open(outBag, rosbag::bagmode::Append);
    double time;
    cv::Mat image;

    for (int i =0; i < vTimeStamp.size(); i++)
    {
        std::stringstream ss;
        ss << std::setfill('0') << std::setw(10) << i;
        std::string imageName = strPathToImage + "/data/" + ss.str() + ".png"; 
        image = cv::imread(imageName, cv::IMREAD_GRAYSCALE);
        time = vTimeStamp[i] + 315964800 + 604800 * gpsWeek - 8 * 3600;

        sensor_msgs::ImagePtr rosImg;
        cv_bridge::CvImage rosImage;
        rosImage.encoding = "mono8";
        rosImage.image = image;

        rosImg = rosImage.toImageMsg();
        rosImg->header.stamp = ros::Time(time);

        bag.write(LorR, ros::Time(time), rosImg);
        std::cout << "done: " << i << "/" << vTimeStamp.size() << std::endl; 
    }
    bag.close();
    std::cout << "image convert finished!" << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_to_rosbag");
    if (argc != 6) {
        ROS_ERROR_STREAM("Usage: data2bag gps_week imu_file left_image_path right_image_path out_bag");
        exit(1);
    }
    ros::NodeHandle nh;

    // 创建rosbag文件
    rosbag::Bag bag;
    int gpsWeek = std::atoi(argv[1]);
    std::string imuFile = argv[2];
    std::string outBag = argv[5];
    std::string LstrPathToImage = argv[3];
    std::string RstrPathToImage = argv[4];
    
    // imu转bag
    imu2bag(bag, imuFile, outBag, gpsWeek);

    // image转bag
    image2bag(bag, LstrPathToImage, outBag, gpsWeek, "/cam0");
    image2bag(bag, RstrPathToImage, outBag, gpsWeek, "/cam1");

    return 0;
}