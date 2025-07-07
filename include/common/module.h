#ifndef MODULE_H
#define MODULE_H

/* C++ Package */
#include <deque>
#include <map>
#include <memory>
#include <iostream>
#include <string>

namespace SimpleSlam {

/* 提前声明 */
class Module;
class System;

/* @brief 子模块基类 
 * @note 子模块是模块的组成部分，用于实现模块的某个算法
 * @note 子模块需要实现init函数，用于初始化子模块，并返回是否初始化成功
 * @note 子模块需要实现get_name函数，用于获取子模块的名称
*/
class SubModule {
public:
    SubModule(std::string strSubModuleName):strSubModuleName_(strSubModuleName){}
    ~SubModule(void){};
    virtual bool init(const System* pSystem,const Module* pModule) = 0;
    virtual std::string get_name()  = 0;
protected:
    std::string strSubModuleName_;
    const System* pSystem_;
    const Module* pModule_;

};

/* @brief 模块基类 
 * @note 模块是系统的组成部分，用于实现系统的某个功能(如回环、前端、后端)
 * @note 模块需要实现init函数，用于初始化模块，并返回是否初始化成功
 * @note 模块需要实现get_name函数，用于获取模块的名称
 * @note 模块需要实现get_submodule函数，用于获取该模块的子模块指针
*/
class Module {
public:
    Module(std::string strModuleName):strModuleName_(strModuleName){}
    ~Module(void){};
    virtual bool init(const System* pSystem) = 0;
    virtual std::string get_name() const = 0;
    virtual SubModule* get_submodule(const std::string& name) const = 0;
protected:
    std::string strModuleName_;
    std::map<std::string, std::unique_ptr<SubModule>> maSubModules_;
    System* pSystem_;
};

class System {
public:
    System(std::string strSystemName):strSystemName_(strSystemName){}
    ~System(void){};
    virtual bool init() = 0;
    virtual std::string get_name() const = 0;
    virtual Module* get_module(const std::string& name) const = 0;
    virtual SubModule* get_submodule(const std::string& module_name, const std::string& submodule_name) const = 0;
protected:
    std::string strSystemName_;
    std::map<std::string, std::unique_ptr<Module>> maModules_;
};
}
#endif // MODULE_H