#ifndef _SIMPLE_SLAM_DATA_MANAGE_H
#define _SIMPLE_SLAM_DATA_MANAGE_H

/* C++ Package */
#include <deque>
#include <map>
#include <memory>
#include <iostream>
#include <string>
#include <atomic>
#include <vector>
#include <mutex>
#include <stdexcept>
#include <cstddef>  // for size_t

namespace SimpleSLAM {

    /* 提前声明 */
    class Module;
    class System;
    class DataManage;

    /**
    * @brief 数据块基类，存储一种数据，并由DataManage模块管理
    * @param strSubModuleName 数据块名称
    * @note 数据块可以是点云数据、IMU数据等，需要重写
    */
    template<typename T>
    class DataBlock {
        public:
            DataBlock(std::string strSubModuleName) : strSubModuleName_(strSubModuleName) {}
            virtual ~DataBlock() = default;
            virtual bool init(const DataManage* pDataManage) = 0;
            std::string get_name() const {
                return strSubModuleName_;
            }

        protected:
            std::string strSubModuleName_;
            const DataManage* pDataManage_; // 数据管理指针
    };

    /**
    * @brief Vector数据块类，使用Vector存储一种数据的指针，并由DataManage模块管理
    * @note 适合大型数据如点云，通过指针避免大对象拷贝
    */
    template<typename T>
    class DataPtrVectorBlock : public DataBlock<T> {
        public:
            DataPtrVectorBlock(std::string strSubModuleName) : DataBlock<T>(strSubModuleName) {}

            bool init(const DataManage* pDataManage) override {
                this->pDataManage_ = pDataManage;
                return true;
            }

            void pushBack(const T* data) {
                std::lock_guard<std::mutex> lock(mutex_);
                if (data_vector_ptr_ == nullptr) {
                    data_vector_ptr_ = std::make_shared<std::vector<const T*>>();
                }
                data_vector_ptr_->push_back(data);
            }

            size_t getSize() const {
                std::lock_guard<std::mutex> lock(mutex_);
                return data_vector_ptr_ ? data_vector_ptr_->size() : 0;
            }

            const T* getData(size_t index) const {
                std::lock_guard<std::mutex> lock(mutex_);
                if (data_vector_ptr_ && index < data_vector_ptr_->size()) {
                    return (*data_vector_ptr_)[index];
                }
                else{
                    throw std::out_of_range("Index out of range");
                }
            }

            void modifyData(size_t index, const T* data) {
                std::lock_guard<std::mutex> lock(mutex_);
                if (data_vector_ptr_ && index < data_vector_ptr_->size()) {
                    (*data_vector_ptr_)[index] = data;
                }
                else{
                    throw std::out_of_range("Index out of range");
                }
            }

            void reset() {
                std::lock_guard<std::mutex> lock(mutex_);
                if (data_vector_ptr_) {
                    data_vector_ptr_->clear();
                }
            }

            std::shared_ptr<std::vector<const T*>> getAllData() const {
                std::lock_guard<std::mutex> lock(mutex_);
                return data_vector_ptr_;
            }

        protected:
            std::shared_ptr<std::vector<const T*>> data_vector_ptr_; // 数据Vector的智能指针
            mutable std::mutex mutex_; // 互斥锁，保护数据Vector的线程安全
    };

    /**
    * @brief Vector数据块类，使用Vector存储一种数据的对象，并由DataManage模块管理
    * @note 适合小型数据如IMU数据、位姿数据等，直接存储对象副本
    */
    template<typename T>
    class DataObjVectorBlock : public DataBlock<T> {
        public:
            DataObjVectorBlock(std::string strSubModuleName) : DataBlock<T>(strSubModuleName) {}

            bool init(const DataManage* pDataManage) override {
                this->pDataManage_ = pDataManage;
                return true;
            }

            void pushBack(const T& data) {
                std::lock_guard<std::mutex> lock(mutex_);
                if (data_vector_obj_ == nullptr) {
                    data_vector_obj_ = std::make_shared<std::vector<T>>();
                }
                data_vector_obj_->push_back(data);
            }

            size_t getSize() const {
                std::lock_guard<std::mutex> lock(mutex_);
                return data_vector_obj_ ? data_vector_obj_->size() : 0;
            }

            T getData(size_t index) const {
                std::lock_guard<std::mutex> lock(mutex_);
                if (data_vector_obj_ && index < data_vector_obj_->size()) {
                    return (*data_vector_obj_)[index];
                }
                else{
                    throw std::out_of_range("Index out of range");
                }
            }

            T& getDataRef(size_t index) {
                std::lock_guard<std::mutex> lock(mutex_);
                if (data_vector_obj_ && index < data_vector_obj_->size()) {
                    return (*data_vector_obj_)[index];
                }
                else{
                    throw std::out_of_range("Index out of range");
                }
            }

            void modifyData(size_t index, const T& data) {
                std::lock_guard<std::mutex> lock(mutex_);
                if (data_vector_obj_ && index < data_vector_obj_->size()) {
                    (*data_vector_obj_)[index] = data;
                }
                else{
                    throw std::out_of_range("Index out of range");
                }
            }

            void reset() {
                std::lock_guard<std::mutex> lock(mutex_);
                if (data_vector_obj_) {
                    data_vector_obj_->clear();
                }
            }

            std::shared_ptr<std::vector<T>> getAllData() const {
                std::lock_guard<std::mutex> lock(mutex_);
                return data_vector_obj_;
            }

        protected:
            std::shared_ptr<std::vector<T>> data_vector_obj_; // 数据Vector的智能指针
            mutable std::mutex mutex_; // 互斥锁，保护数据Vector的线程安全
    };

    /**
    * @brief Deque数据块类，使用Deque存储一种数据的指针，并由DataManage模块管理
    * @note 适合需要频繁头尾操作的大型数据，如滑动窗口的点云数据
    */
    template<typename T>
    class DataPtrDequeBlock : public DataBlock<T> {
        public:
            DataPtrDequeBlock(std::string strSubModuleName, size_t max_size = 1000) 
                : DataBlock<T>(strSubModuleName), max_size_(max_size) {}

            bool init(const DataManage* pDataManage) override {
                this->pDataManage_ = pDataManage;
                return true;
            }

            void pushBack(const T* data) {
                std::lock_guard<std::mutex> lock(mutex_);
                if (data_deque_ptr_ == nullptr) {
                    data_deque_ptr_ = std::make_shared<std::deque<const T*>>();
                }
                data_deque_ptr_->push_back(data);
                
                // 限制队列大小
                if (data_deque_ptr_->size() > max_size_) {
                    data_deque_ptr_->pop_front();
                }
            }

            void pushFront(const T* data) {
                std::lock_guard<std::mutex> lock(mutex_);
                if (data_deque_ptr_ == nullptr) {
                    data_deque_ptr_ = std::make_shared<std::deque<const T*>>();
                }
                data_deque_ptr_->push_front(data);
                
                // 限制队列大小
                if (data_deque_ptr_->size() > max_size_) {
                    data_deque_ptr_->pop_back();
                }
            }

            const T* popFront() {
                std::lock_guard<std::mutex> lock(mutex_);
                if (data_deque_ptr_ && !data_deque_ptr_->empty()) {
                    const T* data = data_deque_ptr_->front();
                    data_deque_ptr_->pop_front();
                    return data;
                }
                return nullptr;
            }

            const T* popBack() {
                std::lock_guard<std::mutex> lock(mutex_);
                if (data_deque_ptr_ && !data_deque_ptr_->empty()) {
                    const T* data = data_deque_ptr_->back();
                    data_deque_ptr_->pop_back();
                    return data;
                }
                return nullptr;
            }

            size_t getSize() const {
                std::lock_guard<std::mutex> lock(mutex_);
                return data_deque_ptr_ ? data_deque_ptr_->size() : 0;
            }

            const T* getData(size_t index) const {
                std::lock_guard<std::mutex> lock(mutex_);
                if (data_deque_ptr_ && index < data_deque_ptr_->size()) {
                    return (*data_deque_ptr_)[index];
                }
                else{
                    throw std::out_of_range("Index out of range");
                }
            }

            void reset() {
                std::lock_guard<std::mutex> lock(mutex_);
                if (data_deque_ptr_) {
                    data_deque_ptr_->clear();
                }
            }

            std::shared_ptr<std::deque<const T*>> getAllData() const {
                std::lock_guard<std::mutex> lock(mutex_);
                return data_deque_ptr_;
            }

        protected:
            std::shared_ptr<std::deque<const T*>> data_deque_ptr_; // 数据Deque的智能指针
            size_t max_size_; // 最大队列长度
            mutable std::mutex mutex_; // 互斥锁，保护数据Deque的线程安全
    };

    /**
    * @brief Deque数据块类，使用Deque存储一种数据的对象，并由DataManage模块管理
    * @note 适合需要频繁头尾操作的小型数据，如IMU数据流、时间戳序列等
    */
    template<typename T>
    class DataObjDequeBlock : public DataBlock<T> {
        public:
            DataObjDequeBlock(std::string strSubModuleName, size_t max_size = 1000) 
                : DataBlock<T>(strSubModuleName), max_size_(max_size) {}

            bool init(const DataManage* pDataManage) override {
                this->pDataManage_ = pDataManage;
                return true;
            }

            void pushBack(const T& data) {
                std::lock_guard<std::mutex> lock(mutex_);
                if (data_deque_obj_ == nullptr) {
                    data_deque_obj_ = std::make_shared<std::deque<T>>();
                }
                data_deque_obj_->push_back(data);
                
                // 限制队列大小
                if (data_deque_obj_->size() > max_size_) {
                    data_deque_obj_->pop_front();
                }
            }

            void pushFront(const T& data) {
                std::lock_guard<std::mutex> lock(mutex_);
                if (data_deque_obj_ == nullptr) {
                    data_deque_obj_ = std::make_shared<std::deque<T>>();
                }
                data_deque_obj_->push_front(data);
                
                // 限制队列大小
                if (data_deque_obj_->size() > max_size_) {
                    data_deque_obj_->pop_back();
                }
            }

            T popFront() {
                std::lock_guard<std::mutex> lock(mutex_);
                if (data_deque_obj_ && !data_deque_obj_->empty()) {
                    T data = data_deque_obj_->front();
                    data_deque_obj_->pop_front();
                    return data;
                }
                throw std::runtime_error("Deque is empty");
            }

            T popBack() {
                std::lock_guard<std::mutex> lock(mutex_);
                if (data_deque_obj_ && !data_deque_obj_->empty()) {
                    T data = data_deque_obj_->back();
                    data_deque_obj_->pop_back();
                    return data;
                }
                throw std::runtime_error("Deque is empty");
            }

            bool tryPopFront(T& data) {
                std::lock_guard<std::mutex> lock(mutex_);
                if (data_deque_obj_ && !data_deque_obj_->empty()) {
                    data = data_deque_obj_->front();
                    data_deque_obj_->pop_front();
                    return true;
                }
                return false;
            }

            bool tryPopBack(T& data) {
                std::lock_guard<std::mutex> lock(mutex_);
                if (data_deque_obj_ && !data_deque_obj_->empty()) {
                    data = data_deque_obj_->back();
                    data_deque_obj_->pop_back();
                    return true;
                }
                return false;
            }

            size_t getSize() const {
                std::lock_guard<std::mutex> lock(mutex_);
                return data_deque_obj_ ? data_deque_obj_->size() : 0;
            }

            T getData(size_t index) const {
                std::lock_guard<std::mutex> lock(mutex_);
                if (data_deque_obj_ && index < data_deque_obj_->size()) {
                    return (*data_deque_obj_)[index];
                }
                else{
                    throw std::out_of_range("Index out of range");
                }
            }

            T& getDataRef(size_t index) {
                std::lock_guard<std::mutex> lock(mutex_);
                if (data_deque_obj_ && index < data_deque_obj_->size()) {
                    return (*data_deque_obj_)[index];
                }
                else{
                    throw std::out_of_range("Index out of range");
                }
            }

            void modifyData(size_t index, const T& data) {
                std::lock_guard<std::mutex> lock(mutex_);
                if (data_deque_obj_ && index < data_deque_obj_->size()) {
                    (*data_deque_obj_)[index] = data;
                }
                else{
                    throw std::out_of_range("Index out of range");
                }
            }

            void reset() {
                std::lock_guard<std::mutex> lock(mutex_);
                if (data_deque_obj_) {
                    data_deque_obj_->clear();
                }
            }

            std::shared_ptr<std::deque<T>> getAllData() const {
                std::lock_guard<std::mutex> lock(mutex_);
                return data_deque_obj_;
            }

        protected:
            std::shared_ptr<std::deque<T>> data_deque_obj_; // 数据Deque的智能指针
            size_t max_size_; // 最大队列长度
            mutable std::mutex mutex_; // 互斥锁，保护数据Deque的线程安全
    };

    /**
    * @brief 数据管理类，负责管理所有数据块，实现数据服务器功能
    * @note 提供注册数据块、获取数据块指针等功能，支持线程安全操作
    */
    class DataManage {
        public:
            DataManage() = default;
            ~DataManage() = default;

            // 禁止拷贝构造和赋值
            DataManage(const DataManage&) = delete;
            DataManage& operator=(const DataManage&) = delete;

            /**
             * @brief 初始化数据管理器
             * @return true 初始化成功，false 初始化失败
             */
            bool init() {
                std::lock_guard<std::mutex> lock(mutex_);
                is_initialized_ = true;
                return true;
            }

            /**
             * @brief 注册数据块到数据管理器
             * @param pDataBlock 数据块指针
             * @return true 注册成功，false 注册失败
             */
            template<typename T>
            bool registerDataBlock(std::shared_ptr<DataBlock<T>> pDataBlock) {
                if (!pDataBlock) {
                    std::cerr << "[DataManage] 错误：尝试注册空的数据块指针" << std::endl;
                    return false;
                }

                std::lock_guard<std::mutex> lock(mutex_);
                
                std::string blockName = pDataBlock->get_name();
                
                // 检查是否已经注册了同名的数据块
                if (data_blocks_.find(blockName) != data_blocks_.end()) {
                    std::cerr << "[DataManage] 警告：数据块 '" << blockName << "' 已存在，将被覆盖" << std::endl;
                }

                // 初始化数据块
                if (!pDataBlock->init(this)) {
                    std::cerr << "[DataManage] 错误：数据块 '" << blockName << "' 初始化失败" << std::endl;
                    return false;
                }

                // 注册数据块（使用 void* 存储以支持不同模板类型）
                data_blocks_[blockName] = std::static_pointer_cast<void>(pDataBlock);
                
                std::cout << "[DataManage] 成功注册数据块：" << blockName << std::endl;
                return true;
            }

            /**
             * @brief 获取指定名称的数据块指针
             * @param blockName 数据块名称
             * @return 数据块指针，如果不存在返回 nullptr
             */
            template<typename T>
            std::shared_ptr<DataBlock<T>> getDataBlock(const std::string& blockName) const {
                std::lock_guard<std::mutex> lock(mutex_);
                
                auto it = data_blocks_.find(blockName);
                if (it == data_blocks_.end()) {
                    std::cerr << "[DataManage] 错误：未找到数据块 '" << blockName << "'" << std::endl;
                    return nullptr;
                }

                // 将 void* 转换回具体的数据块类型
                auto pDataBlock = std::static_pointer_cast<DataBlock<T>>(it->second);
                return pDataBlock;
            }

            /**
             * @brief 获取指定类型的数据块指针（便捷方法）
             * @param blockName 数据块名称
             * @return 具体类型的数据块指针，如果不存在或类型不匹配返回 nullptr
             */
            template<typename DataBlockType>
            std::shared_ptr<DataBlockType> getTypedDataBlock(const std::string& blockName) const {
                std::lock_guard<std::mutex> lock(mutex_);
                
                auto it = data_blocks_.find(blockName);
                if (it == data_blocks_.end()) {
                    std::cerr << "[DataManage] 错误：未找到数据块 '" << blockName << "'" << std::endl;
                    return nullptr;
                }

                // 尝试转换为指定的数据块类型
                auto pDataBlock = std::dynamic_pointer_cast<DataBlockType>(it->second);
                if (!pDataBlock) {
                    std::cerr << "[DataManage] 错误：数据块 '" << blockName << "' 类型转换失败" << std::endl;
                    return nullptr;
                }

                return pDataBlock;
            }

            /**
             * @brief 检查数据块是否存在
             * @param blockName 数据块名称
             * @return true 存在，false 不存在
             */
            bool hasDataBlock(const std::string& blockName) const {
                std::lock_guard<std::mutex> lock(mutex_);
                return data_blocks_.find(blockName) != data_blocks_.end();
            }

            /**
             * @brief 移除指定的数据块
             * @param blockName 数据块名称
             * @return true 移除成功，false 数据块不存在
             */
            bool removeDataBlock(const std::string& blockName) {
                std::lock_guard<std::mutex> lock(mutex_);
                
                auto it = data_blocks_.find(blockName);
                if (it == data_blocks_.end()) {
                    std::cerr << "[DataManage] 错误：尝试移除不存在的数据块 '" << blockName << "'" << std::endl;
                    return false;
                }

                data_blocks_.erase(it);
                std::cout << "[DataManage] 成功移除数据块：" << blockName << std::endl;
                return true;
            }

            /**
             * @brief 获取所有已注册的数据块名称
             * @return 数据块名称列表
             */
            std::vector<std::string> getAllDataBlockNames() const {
                std::lock_guard<std::mutex> lock(mutex_);
                std::vector<std::string> names;
                names.reserve(data_blocks_.size());
                
                for (const auto& pair : data_blocks_) {
                    names.push_back(pair.first);
                }
                
                return names;
            }

            /**
             * @brief 获取已注册数据块的数量
             * @return 数据块数量
             */
            size_t getDataBlockCount() const {
                std::lock_guard<std::mutex> lock(mutex_);
                return data_blocks_.size();
            }

            /**
             * @brief 清空所有数据块
             */
            void clear() {
                std::lock_guard<std::mutex> lock(mutex_);
                data_blocks_.clear();
                std::cout << "[DataManage] 已清空所有数据块" << std::endl;
            }

            /**
             * @brief 打印所有已注册的数据块信息
             */
            void printDataBlockInfo() const {
                std::lock_guard<std::mutex> lock(mutex_);
                std::cout << "[DataManage] 已注册的数据块列表：" << std::endl;
                if (data_blocks_.empty()) {
                    std::cout << "  (无数据块)" << std::endl;
                    return;
                }

                for (const auto& pair : data_blocks_) {
                    std::cout << "  - " << pair.first << std::endl;
                }
            }

            /**
             * @brief 检查数据管理器是否已初始化
             * @return true 已初始化，false 未初始化
             */
            bool isInitialized() const {
                std::lock_guard<std::mutex> lock(mutex_);
                return is_initialized_;
            }

        private:
            // 使用 map 存储数据块，key 为数据块名称，value 为 void* 指针
            std::map<std::string, std::shared_ptr<void>> data_blocks_;
            mutable std::mutex mutex_; // 互斥锁，保护数据块映射的线程安全
            std::atomic<bool> is_initialized_{false}; // 初始化状态
    };

}
#endif // _SIMPLE_SLAM_DATA_MANAGE_H 