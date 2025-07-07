#ifndef SENSOR_IO_H
#define SENSOR_IO_H

/* C++ Package */
#include <vector>
#include <deque>
#include <iostream>
#include <string>
#include <mutex>
#include <thread>
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
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/opencv.hpp>
/* 自定义 */
#include<common/data_type.h>
#include<common/module.h>



namespace SimpleSlam {

/*
 * @brief 传感器IO基类，作为所有传感器IO模块的基类
 * @note 该类是一个抽象类，用于处理单一传感器消息，将其转换为传感器数据，并提供数据读取接口，需要重写纯虚函数processMsgCbk、popData、pushData，分别实现消息处理、数据读取和数据写入。
*/
template<typename SensorMsgType, typename SensorDataType>
class SensorSubModule : public SimpleSlam::SubModule {
public:
    SensorSubModule(std::string strSubModuleName):SimpleSlam::SubModule(strSubModuleName){};
    ~SensorSubModule(void){};
    virtual bool init(const SimpleSlam::System* pSystem,const SimpleSlam::Module* pModule) = 0;
    virtual std::string get_name() = 0;
    /* 数据处理接口，需要重写 */
    virtual SensorDataType popData() = 0;
    virtual void pushData(const SensorDataType& data) = 0;
    virtual void processMsgCbk(const SensorMsgType& msg) = 0;


protected:
    std::deque<SensorDataType> deqData_;
    std::mutex mtxData_;
    std::thread thread_;
    std::string strSubscribeTopic_;

};

/*
 * @brief IMU传感器子模块实现范例，负责处理IMU信息，接受sensor_msgs::Imu消息，并转换为Pose6D数据
*/
class IMUSubModule : public SensorSubModule<sensor_msgs::Imu, Pose6D> {
public:
    IMUSubModule(std::string strSubModuleName);
    ~IMUSubModule(void);
    bool init(const SimpleSlam::System* pSystem,const SimpleSlam::Module* pModule) override;
    std::string get_name() override;
    void processMsgCbk(const sensor_msgs::Imu& msg) override;
    Pose6D popData() override;
    void pushData(const Pose6D& data) override;


protected:
    std::string strImuTopic_;
    ros::NodeHandle nh_;
    ros::Subscriber subscriberImu_;
    int num = 0;
};

inline IMUSubModule::IMUSubModule(std::string strSubModuleName):SensorSubModule(strSubModuleName){
    nh_.param<std::string>("imu_topic", strImuTopic_, "/camera/imu");
    subscriberImu_ = nh_.subscribe(strImuTopic_, 1000, &IMUSubModule::processMsgCbk, this);
}

inline IMUSubModule::~IMUSubModule(void){
    if(subscriberImu_){
        subscriberImu_.shutdown();
    }
}

inline void IMUSubModule::pushData(const Pose6D& data){
    std::lock_guard<std::mutex> lock(mtxData_);
    deqData_.push_back(data);
}

inline Pose6D IMUSubModule::popData(){
    std::lock_guard<std::mutex> lock(mtxData_);
    if(deqData_.empty()){
        return Pose6D();
    }
    Pose6D pose6D = deqData_.front();
    deqData_.pop_front();
    return pose6D;
}

inline void IMUSubModule::processMsgCbk(const sensor_msgs::Imu& msg){
    Pose6D pose6D;
    pose6D.time = msg.header.stamp.toSec();
    pose6D.acc = V3D(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
    pose6D.gyr = V3D(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
    pushData(pose6D);
    num++;
    if(num % 100 == 0){
        ROS_INFO("IMU SubModule: Processed %d messages", num);
    }
}

inline std::string IMUSubModule::get_name() {
    return strSubModuleName_;
}


inline bool IMUSubModule::init(const SimpleSlam::System* pSystem,const SimpleSlam::Module* pModule){
    this->pSystem_ = pSystem;
    this->pModule_ = pModule;
    return true;
}
}
#endif