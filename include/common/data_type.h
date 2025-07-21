#ifndef _SIMPLE_SLAM_DATA_TYPE_H
#define _SIMPLE_SLAM_DATA_TYPE_H
#include<concepts>
#include <Eigen/Dense>
#include<pcl-1.12/pcl/point_cloud.h>
typedef Eigen::Matrix<double, 3, 1> V3D; // 三维向量
typedef Eigen::Matrix<double, 3, 3> M3D; // 三维矩阵
typedef Eigen::Matrix<double, 4, 4> M4D; // 四维矩阵


/*
 * @brief 针对IMU数据的6D位姿结构体
 * @param time: 时间戳
 * @param acc: 加速度
 * @param gyr: 角速度   
 * @param vel: 预积分得出的速度
 * @param pos: 预积分得出的位置
 * @param rot: 预积分得出的旋转矩阵
 */
struct Pose6D
{
    Pose6D() : time(0), acc(V3D::Zero()), gyr(V3D::Zero()), vel(V3D::Zero()), pos(V3D::Zero()), rot(M3D::Identity()) {}
    Pose6D(double t, const V3D &a, const V3D &g, const V3D &v, const V3D &p, const M3D &r)
        : time(t), acc(a), gyr(g), vel(v), pos(p), rot(r) {}
    double time; // 时间戳
    V3D acc; // 加速度
    V3D gyr; // 角速度
    V3D vel; // 速度
    V3D pos; // 位置
    M3D rot; // 旋转矩阵
};

/*
 * @brief 点云数据结构体
 * @param x: x坐标
 * @param y: y坐标
 * @param z: z坐标
 * @param intensity: 强度
 * @param timestamp: 时间戳
*/
struct CTPoint
{
    CTPoint() : timestamp(0), x(0), y(0), z(0), intensity(0) {}
    CTPoint(double t, double x, double y, double z) : timestamp(t), x(x), y(y), z(z) {}
    double timestamp;
    double x;
    double y;
    double z;
    double intensity;
};


#endif