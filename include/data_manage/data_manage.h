#ifndef DATA_MANAGE_H
#define DATA_MANAGE_H

#include <common/data_type.h>

class DataManager{
public:
    DataManager() = default;

    /**
     * @brief 处理IMU数据
     * @tparam T IMU数据类型，必须满足IMUData概念
     * @param imu_data IMU数据
     */
    template<typename T>
    void processIMUData(const T& imu_data) {
        // 处理IMU数据的逻辑
        // 例如：存储、预处理等
    }

    // 其他数据处理方法...
};
#endif