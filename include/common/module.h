#ifndef _SIMPLE_SLAM_MODULE_H
#define _SIMPLE_SLAM_MODULE_H

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

namespace SimpleSLAM {

    /* 提前声明 */
    class Module;
    class System;
    class DataManage;

    /* @brief 子模块基类 
    * @note 子模块是模块的组成部分，用于实现模块的某个算法
    * @note 子模块需要实现init、stop、run函数
    */
    class SubModule {
    public:
        SubModule(std::string strSubModuleName):strSubModuleName_(strSubModuleName){}
        ~SubModule(void){};
        // @brief 初始化子模块，同时传入上层模块指针
        // @param pModule 模块指针
        // @return 是否初始化成功
        virtual bool init(const Module* pModule) = 0;
        std::string get_name() {return strSubModuleName_;};
        virtual void run() = 0;
        virtual void stop() = 0;
    protected:
        std::string strSubModuleName_;
        std::atomic<bool> is_active_;
        const Module* pModule_;
        const DataManage* pDataManage_;
    };

    /* @brief 模块基类 
    * @note 模块是系统的组成部分，用于实现系统的某个功能(如回环、前端、后端)
    */
    class Module {
        public:
            Module(std::string strModuleName):strModuleName_(strModuleName){}
            ~Module(void){};
            virtual bool init(const System* pSystem) = 0;
            std::string get_name() const {return strModuleName_;};
            DataManage* get_data_manage() const {
                if (pDataManage_ == nullptr) {
                    throw std::runtime_error("DataManage is not initialized");
                }
                return pDataManage_;
            }
            virtual SubModule* get_submodule(const std::string& name) const = 0;
            virtual void register_submodule(const std::string& name, SubModule* pSubModule) = 0;
            virtual void run() = 0;
            virtual void stop() = 0;
        protected:
            std::string strModuleName_;
            std::atomic<bool> is_active_;
            std::map<std::string, std::unique_ptr<SubModule>> mapSubModules_;
            System* pSystem_;
            DataManage* pDataManage_;
    };

    /* @brief 系统基类 
    * @note 系统是模块的集合，用于实现系统功能
    */
    class System {
        public:
            System(std::string strSystemName):strSystemName_(strSystemName){}
            ~System(void){};
            virtual bool init() = 0;
            virtual std::string get_name() const = 0;
            virtual Module* get_module(const std::string& name) const = 0;
            virtual SubModule* get_submodule(const std::string& module_name, const std::string& submodule_name) const = 0;
            virtual void register_module(const std::string& name, Module* pModule) = 0;
            virtual void register_submodule(const std::string& module_name, const std::string& submodule_name, SubModule* pSubModule) = 0;
            virtual void run() = 0;
            virtual void stop() = 0;
        protected:
            std::string strSystemName_;
            std::atomic<bool> is_active_;
            std::map<std::string, std::unique_ptr<Module>> mapModules_;
    };

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
                std::lock_guard<std::mutex> lock(mutex_); // 确保线程安全
                if (data_vector_ptr_) {
                    data_vector_ptr_->clear();
                }
            }

            std::shared_ptr<std::vector<const T*>> getAllData() const {
                std::lock_guard<std::mutex> lock(mutex_);
                return data_vector_ptr_;
            }

        protected:
            std::shared_ptr<std::vector<const T*>> data_vector_ptr_ = std::make_shared<std::vector<const T*>>(); // 数据Vector的智能指针
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
            std::shared_ptr<std::vector<T>> data_vector_obj_ = new std::vector<T>(); // 数据Vector的智能指针
            mutable std::mutex mutex_; // 互斥锁，保护数据Vector的线程安全
    };

    /**
    * @brief Deque数据块类，使用Deque存储一种数据的指针，并由DataManage模块管理
    * @note 适合需要频繁头尾操作的大型数据，如滑动窗口的点云数据
    */
    template<typename T>
    class DataPtrDequeBlock : public DataBlock<T> {
        public:
            DataPtrDequeBlock(std::string strSubModuleName) : DataBlock<T>(strSubModuleName){}

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
                
            }

            void pushFront(const T* data) {
                std::lock_guard<std::mutex> lock(mutex_);
                if (data_deque_ptr_ == nullptr) {
                    data_deque_ptr_ = std::make_shared<std::deque<const T*>>();
                }
                data_deque_ptr_->push_front(data);
                
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
            std::shared_ptr<std::deque<const T*>> data_deque_ptr_ = std::make_shared<std::deque<const T*>>(); // 数据Deque的智能指针
            mutable std::mutex mutex_; // 互斥锁，保护数据Deque的线程安全
    };

    /**
    * @brief Deque数据块类，使用Deque存储一种数据的对象，并由DataManage模块管理
    * @note 适合需要频繁头尾操作的小型数据，如IMU数据流、时间戳序列等
    */
    template<typename T>
    class DataObjDequeBlock : public DataBlock<T> {
        public:
            DataObjDequeBlock(std::string strSubModuleName) : DataBlock<T>(strSubModuleName) {}

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
            }

            void pushFront(const T& data) {
                std::lock_guard<std::mutex> lock(mutex_);
                if (data_deque_obj_ == nullptr) {
                    data_deque_obj_ = std::make_shared<std::deque<T>>();
                }
                data_deque_obj_->push_front(data);
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
            mutable std::mutex mutex_; // 互斥锁，保护数据Deque的线程安全
    };

}
#endif // _SIMPLE_SLAM_MODULE_H