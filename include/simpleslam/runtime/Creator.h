// CreateByType.hpp (或直接放到你的 SimpleSLAMRegistry.hpp 里)
#pragma once

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "core/Registry.hpp" // 需要：SimpleSLAM::Params, SimpleSLAM::RegistryFactory

namespace SimpleSLAM {

// 可选：用于更明确的异常类型（便于上层捕获区分）
class CreateError final : public std::runtime_error {
public:
  explicit CreateError(const std::string& msg) : std::runtime_error(msg) {}
};

// 1) 抛异常版本：失败直接抛 CreateError（推荐用于框架/Builder）
template <typename Base>
std::unique_ptr<Base> Create(
    const std::string& type,
    std::string instance_name,
    Params params)
{
  try {
    // Base 决定注册表：RegistryFactory<Base> 必须存在并已注册 type
    return RegistryFactory<Base>::Instance().Create(
        type,
        std::move(instance_name),
        std::move(params));
  } catch (const std::exception& e) {
    // 这里不检查 params，只在错误时补充上下文，便于定位
    throw CreateError(
        std::string("CreateByType failed: base=") + typeid(Base).name() +
        ", type='" + type + "', instance_name='" + instance_name + "'. " +
        "Reason: " + e.what());
  }
}

// 2) 不抛异常版本：失败返回 nullptr，并把错误写入 err（适合你想继续跑的场景）
template <typename Base>
std::unique_ptr<Base> TryCreate(
    const std::string& type,
    std::string instance_name,
    Params params,
    std::string* err = nullptr) noexcept
{
  try {
    return RegistryFactory<Base>::Instance().Create(
        type,
        std::move(instance_name),
        std::move(params));
  } catch (const std::exception& e) {
    if (err) *err = e.what();
    return nullptr;
  }
}

} // namespace SimpleSLAM
