#ifndef _SIMPLE_SLAM_ESKF_H
#define _SIMPLE_SLAM_ESKF_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <deque>
#include <map>
#include <iostream>
#include <string>
#include <mutex>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <sophus/se3.hpp>
#include "common/module.h"
#include "common/data_type.h"

template<typename IMUDataType>
class ESKF : public SimpleSLAM::SubModule {
public:
    ESKF();
    ~ESKF();

private:
    M3D Acc;//加速度
    M3D Gyr;//角速度
    M3D AccBias;//加速度偏置
    M3D GyrBias;//角速度偏置
    M3D AccNoise;//加速度噪声
    M3D GyrNoise;//角速度噪声
    M3D AccBiasNoise;//加速度偏置噪声
    M3D GyrBiasNoise;//角速度偏置噪声

    
    V3D Vel;//速度
    V3D Pos;//位置
    M3D Rot;//旋转矩阵
    V3D Gravity;//重力

    M3D RotLidar2Body;
    V3D PosLidar2Body;

    int imuPreIntegrateSize;//IMU预积分窗口大小

    Sophus::SO3<double> SO3Rot;
private:
    ros::NodeHandle nh_;
    std::map<std::string,std::shared_ptr<SimpleSLAM::SubModule>> mapSubModules_;
    void EstimateInitState();

    void PreIntegrate();

    void Predict();
    void PredictOnce(const IMUDataType& imuData);

    void Update();

    void Correct();

    void GetState();
};

/*
 * @brief 初始化参数，包括加速度和加速度的噪声、偏置和偏置噪声，参数初始值通过ros参数服务器获取
 * @param 无
 * @return 无
 */
template<typename IMUDataType>
void ESKF<IMUDataType>::InitParamfromRos()
{
    double temp;
    nh_.param<vector<double>>("AccNoise", AccNoise, vector<double>(3, 0.01));
    nh_.param<vector<double>>("GyrNoise", GyrNoise, vector<double>(3, 0.01));
    nh_.param<vector<double>>("AccBiasNoise", AccBiasNoise, vector<double>(3, 0.01));
    nh_.param<vector<double>>("GyrBiasNoise", GyrBiasNoise, vector<double>(3, 0.01));
    nh_.param<vector<double>>("AccBiasAcc", AccBiasAcc, vector<double>(3, 0.01));
    nh_.param<vector<double>>("GyrBiasAcc", GyrBiasAcc, vector<double>(3, 0.01));
    
    Vel >> 0,0,0;
    Pos >> 0,0,0;
    Rot = M3D::Identity();

    nh_.param<vector<double>>("RotLidar2Body", RotLidar2Body, M3D::Identity());
    nh_.param<vector<double>>("PosLidar2Body", PosLidar2Body, vector<double>(3, 0.0));

}

/*
 * @brief 计算IMU的预积分，获取其偏置
 * @param imuData 特定IMU数据结构体的队列
 * @return 无
 */
template<typename IMUDataType>
void ESKF<IMUDataType>::PreIntegrate(const std::deque<IMUDataType>& imuData)
{
    imt imuNum = 0;
    V3D meanLinearAcc = V3D::Zero(), meanAngularVel = V3D::Zero();
    for(const auto& imu : imuData)
    {
        meanLinearAcc += (imu.acc - meanLinearAcc) / (imuNum + 1);
        meanAngularVel += (imu.gyr - meanAngularVel) / (imuNum + 1);
        imuNum++;
    }
    
}

template<typename IMUDataType>
void ESKF<IMUDataType>::Predict()
{}
#endif // ESKF_H