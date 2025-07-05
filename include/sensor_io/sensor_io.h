#ifndef SENSOR_IO_H
#define SENSOR_IO_H

#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>
#include <numeric>
#include <Eigen/Dense>
/* ROS Package */
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include<livox_ros_driver/CustomMsg.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

/* 自定义 */
#include<common/data_type.h>

/*
 * @brief 建图模式：离线或在线
 * @note 离线建图通常是指从rosbag文件中读取数据进行建图，
 *       在线建图则是指实时从传感器（如激光雷达、IMU、相机等）话题获取数据进行建图。
 */
enum class MappingMode
{
    kOffline=0,
    kOnline=1,
};

/*
 * @brief 激光雷达类型，包含LIVOX、OUSTER、VELODYNE和MID360等类型
 */
enum class LidarType
{
    LIVOX = 0,
    OUSTER = 1,
    VELODYNE = 2,
    MID360 = 3,
};

/**
 * @brief 传感器IO配置参数结构体
 * @param strLidarTopic: 激光雷达话题名称
 * @param strImuTopic: IMU话题名称
 * @param strCameraTopic: 相机话题名称
 * @param strBagPath: rosbag文件路径
 * @param objMappingMode: 建图模式（在线建图或离线建图）
 * @note 该结构体用于存储传感器相关的配置参数
 */
struct SensorCfg
{
    std::string strLidarTopic;
    std::string strImuTopic;
    std::string strCameraTopic;
    std::string strBagPath;
    MappingMode objMappingMode;
    LidarType objLidarType;
};

class SensorIO
{
public:
    SensorIO(const SensorCfg &cfg);
    ~SensorIO(void);
    void run();
    SensorCfg initCfg();

private:
    void StandardPclCbk(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void LivoxLidarCbk(const livox_ros_driver::CustomMsg::ConstPtr &msg);
    void IMUCbk(const sensor_msgs::Imu::ConstPtr &msg);
    void CameraCbk(const sensor_msgs::Image::ConstPtr &msg);
    void ReadBagData(const std::string &strBagPath);

private:
    ros::NodeHandle nh_;
    /* 在线建图 */
    ros::Subscriber objLidarSub_;
    ros::Subscriber objImuSub_;
    ros::Subscriber objCameraSub_;
    ros::Subscriber objLivoxLidarSub_= nh_.subscribe<livox_ros_driver::CustomMsg>(
        "/livox/lidar", 1000, &SensorIO::LivoxLidarCbk, this);
    /* 离线建图（基于rosbag） */
    rosbag::View objBagView_;
    std::shared_ptr<rosbag::Bag> pobjBag_;

    SensorCfg stSensorIOCfg;

    std::deque<pcl::PointCloud<pcl::PointXYZI>> dqeLidarBuffer_;

    std::deque<Pose6D> dqeImuBuffer_; //IMU数据队列
    bool b_first_imu_ = false; // 是否是第一个IMU数据
    Pose6D last_imu_pose_; // 上一帧IMU位姿

};

#endif