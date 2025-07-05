#include "sensor_io.h"
#include"common/data_type.h"

/* 
* @brief 读取ros参数，初始化传感器配置
* @return SensorCfg 传感器配置结构体
*/
SensorCfg SensorIO::initCfg(){
    struct SensorCfg cfg;
    std::string strMappingMode;
    nh_.param<std::string>("lidar_topic", cfg.strLidarTopic, "/lidar_topic");
    nh_.param<std::string>("camera_topic", cfg.strCameraTopic, "/camera_topic");
    nh_.param<std::string>("imu_topic", cfg.strImuTopic, "/imu_topic");
    nh_.param<std::string>("bag_path", cfg.strBagPath, "/bag/rosbag.bag");
    nh_.param<std::string>("mapping_mode", strMappingMode, "offline");

    if (strMappingMode == "offline") {
        cfg.objMappingMode = MappingMode::kOffline;
    } else {
        cfg.objMappingMode = MappingMode::kOnline;
    }
    return cfg;
}

SensorIO::SensorIO(const SensorCfg &cfg){
    ROS_INFO("传感器开始初始化...");
    stSensorIOCfg = initCfg();
    switch (stSensorIOCfg.objMappingMode) 
    {
        case MappingMode::kOffline:
            ROS_INFO("当前为离线建图模式");
            ReadBagData(stSensorIOCfg.strBagPath);
            break;
        case MappingMode::kOnline:
            ROS_INFO("当前为在线建图模式");
            switch(stSensorIOCfg.objLidarType) {
                case LidarType::LIVOX:
                    ROS_INFO("当前激光雷达类型为LIVOX");
                    break;
                case LidarType::OUSTER:
                    ROS_INFO("当前激光雷达类型为OUSTER");
                    break;
                case LidarType::VELODYNE:
                    ROS_INFO("当前激光雷达类型为VELODYNE");
                    break;
                case LidarType::MID360:
                    ROS_INFO("当前激光雷达类型为MID360");
                    break;
                default:
                    ROS_ERROR("未定义激光雷达类型");
                    throw std::runtime_error("未定义激光雷达类型，需要在配置文件中设置");
                    break;
            }
        objLidarSub_ = nh_.subscribe(stSensorIOCfg.strLidarTopic, 1000, &SensorIO::StandardPclCbk, this);
    }
}

/* 
* @brief IMU回调函数，将IMU数据转换为Pose6D对象并存储到队列
* @param msg IMU消息指针
*/
void SensorIO::IMUCbk(const sensor_msgs::Imu::ConstPtr &msg)
{
    if(!b_first_imu_) {
        b_first_imu_ = true; // 第一次接收到IMU数据
        Pose6D pose(msg->header.stamp.toSec(),
                    V3D(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z),
                    V3D(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
                    V3D::Zero(), // 速度初始化为零
                    V3D::Zero(), // 位置初始化为零
                    M3D::Identity()); // 旋转矩阵初始化为单位矩阵
        dqeImuBuffer_.push_back(pose); // 将第一个IMU数据添加到缓冲区
        last_imu_pose_ = pose; // 保存第一个IMU数据的位姿
        return; // 只处理第一个IMU数据
    }
    double current_time_ = msg->header.stamp.toSec(); // 获取当前IMU数据的时间戳
    V3D current_acc_  = V3D(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    V3D current_angvel_ = V3D(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    Pose6D pose(current_time_,
                current_acc_, // 当前IMU数据的加速度
                current_angvel_, // 当前IMU数据的
                V3D::Zero(), // 速度初始化为零
                V3D::Zero(), // 位置初始化为零
                M3D::Identity()); // 旋转矩阵初始化为单位矩阵
    dqeImuBuffer_.push_back(pose);
}