#ifndef _SIMPLE_SLAM_SENSOR_IO_H
#define _SIMPLE_SLAM_SENSOR_IO_H

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
#include<common/data_manage.h>



namespace SimpleSLAM {
    class SensorIOSubModule : public SimpleSLAM::SubModule {
        public:
            SensorIOSubModule(std::string strSubModuleName) : SubModule(strSubModuleName) {};
            ~SensorIOSubModule() = default;
            bool init(const SimpleSLAM::Module* pModule) override {
                this->pModule_ = pModule;
                this->pDataManage_ = pModule->get_data_manage();
                this->pDataManage_->registerDataBlock(std::make_shared<DataPtrVectorBlock<CTPoint>>("CTPointCloud"));
                return true;
            }
            void PointCloudCbk(const sensor_msgs::PointCloud2ConstPtr& msg) {
                // 处理点云数据
                pcl::PointCloud<pcl::PointXYZ> cloud;
                pcl::fromROSMsg(*msg, cloud);
        private:
            ros::NodeHandle nh_; // ROS节点句柄
            ros::Subscriber objLidarSub_; // 激光雷达订阅者
    };
}
#endif