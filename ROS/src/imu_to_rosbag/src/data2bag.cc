#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/Imu.h>
#include <fstream>
#include <sstream>



int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_to_rosbag");
    ros::NodeHandle nh;

    // 打开文本文件
    std::ifstream file(argv[1]);
    if (!file.is_open())
    {
        ROS_ERROR_STREAM("Failed to open file!");
        return 1;
    }

    // 创建rosbag文件
    rosbag::Bag bag;
    // Write就是写一个新的，在现有基础上加就用Append
    bag.open("./test.bag", rosbag::bagmode::Write);

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
        time = time + 315964800+604800 * 2258- 8 * 3600;
        imu_msg.header.stamp = ros::Time(time);
        imu_msg.angular_velocity.x = gyro_x;
        imu_msg.angular_velocity.y = gyro_y;
        imu_msg.angular_velocity.z = gyro_z;
        imu_msg.linear_acceleration.x = accel_x;
        imu_msg.linear_acceleration.y = accel_y;
        imu_msg.linear_acceleration.z = accel_z;

        // 写入ROSbag文件
        bag.write("imu0", ros::Time(time), imu_msg);
    }

    bag.close();
    file.close();

    return 0;
}