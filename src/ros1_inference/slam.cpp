#include<sensor_io/sensor_io.h>
#include<ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "slam_node");
    ros::NodeHandle nh;

    SensorIO sensor_io();

    // 进入循环，等待回调函数处理数据
    ros::spin();

    return 0;
}