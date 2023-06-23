关于C++转换rosbag数据的程序使用说明
====

## 初始化ROS空间
ROS文件夹下必须有src文件夹/<br>
随后在该文件夹下执行`catkin_create_pkg data_to_rosbag roscpp rosbag`命令/<br>
这时，src文件夹中会创建好data_to_rosbag文件夹，里面包括include、src和一个CMakeLists.txt文件模板/<br>
## 配置文件
将data2bag.cc文件放在data_to_rosbag文件夹的src文件夹中/<br>
并用代码中提供的CMakeLists.txt替换CMakeLists.txt模板/<br>
## 编译运行
只需要进入到ROS文件夹下执行`catkin_make`命令即可/<br>